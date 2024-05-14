/*
 * Copyright (c) 2012 Nicolas George
 * Copyright (c) 2024 Yigithan Yigit - 32 Bit Float Audio Support
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/channel_layout.h"
#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "audio.h"
#include "avfilter.h"
#include "internal.h"

#define MAX_DB_FLT 1024
#define MAX_DB 91
#define HISTOGRAM_SIZE 0x10000
#define HISTOGRAM_SIZE_FLT (MAX_DB_FLT*2)

typedef struct VolDetectContext {
    uint64_t* histogram;            ///< for integer number of samples at each PCM value, for float number of samples at each dB
    uint64_t nb_samples;            ///< number of samples
    size_t histogram_size;          ///< size of the histogram
    double sum2;                    ///< sum of the squares of the samples
    double max;                     ///< maximum sample value
    enum AVSampleFormat sample_fmt; ///< sample format
    void (*process_samples)(struct VolDetectContext *vd, AVFrame *samples);
} VolDetectContext;

static inline double logdb(double v, const int max_db)
{
    if (!v)
        return max_db;

    return -log10(v) * 10;
}

#define PROCESS_SAMPLES(name, type, update_func)                                  \
static void process_samples_##name(VolDetectContext *vd, AVFrame *samples)        \
{                                                                                 \
    const type *p = (const type *)samples->extended_data[0];                      \
    const int nb_samples = samples->nb_samples * samples->ch_layout.nb_channels;  \
    int i;                                                                        \
    for (i = 0; i < nb_samples; i++)                                              \
        update_func(vd, p, i);                                                    \
}

#define PROCESS_SAMPLES_PLANAR(name, type, update_func )                          \
static void process_samples_planar_##name(VolDetectContext *vd, AVFrame *samples) \
{                                                                                 \
    const int channels = samples->ch_layout.nb_channels;                          \
    const int nb_samples = samples->nb_samples;                                   \
    int ch, i;                                                                    \
    for (ch = 0; ch < channels; ch++) {                                           \
        const type *p = (const type *)samples->extended_data[ch];                 \
            for (i = 0; i < nb_samples; i++)                                      \
                update_func(vd, p, i);                                            \
    }                                                                             \
}

#define UPDATE_FLOAT_STATS(vd,p, i )                                              \
do {                                                                              \
    double sample, power;                                                         \
    int idx;                                                                      \
    if(!isfinite(p[i]))                                                           \
        continue;                                                                 \
    sample = FFABS(p[i]);                                                         \
    vd->max = FFMAX(vd->max, sample);                                             \
    power = sample * sample;                                                      \
    vd->sum2 += power;                                                            \
    idx = (int)logdb(power, MAX_DB_FLT) + MAX_DB_FLT;                             \
    vd->histogram[idx]++;                                                         \
    vd->nb_samples++;                                                             \
} while(0)

#define UPDATE_INT_STATS(vd, p, i)                                                \
do {                                                                              \
    vd->histogram[p[i] + 0x8000]++;                                               \
    vd->nb_samples++;                                                             \
} while(0)

PROCESS_SAMPLES(flt, float, UPDATE_FLOAT_STATS)
PROCESS_SAMPLES(s16, int16_t, UPDATE_INT_STATS)

PROCESS_SAMPLES_PLANAR(flt, float, UPDATE_FLOAT_STATS)
PROCESS_SAMPLES_PLANAR(s16, int16_t,UPDATE_INT_STATS)

static int filter_frame(AVFilterLink *inlink, AVFrame *samples)
{
    AVFilterContext *ctx = inlink->dst;
    VolDetectContext *vd = ctx->priv;

    vd->process_samples(vd, samples);

    return ff_filter_frame(inlink->dst->outputs[0], samples);
}

static av_cold void print_stats(AVFilterContext *ctx)
{
    VolDetectContext *vd = ctx->priv;
    int i, max_volume, shift;
    uint64_t nb_samples = 0, power = 0, nb_samples_shift = 0, sum = 0;
    uint64_t histdb[MAX_DB + 1] = { 0 };

    for (i = 0; i < vd->histogram_size; i++)
        nb_samples += vd->histogram[i];

    if (!nb_samples)
        return;

    av_log(ctx, AV_LOG_INFO, "n_samples: %"PRId64"\n", nb_samples);
    switch(vd->sample_fmt) {
        case AV_SAMPLE_FMT_FLTP:
        case AV_SAMPLE_FMT_FLT:
            av_log(ctx, AV_LOG_INFO, "mean_volume: %.1f dB\n", -logdb(vd->sum2 / vd->nb_samples, MAX_DB_FLT));
            av_log(ctx, AV_LOG_INFO, "max_volume: %.1f dB\n", -logdb(vd->max * vd->max, MAX_DB_FLT));
            for (i = 0; i < HISTOGRAM_SIZE_FLT && !vd->histogram[i]; i++);
            for (; i < HISTOGRAM_SIZE_FLT && sum < vd->nb_samples / 1000; i++) {
                if (!vd->histogram[i])
                    continue;
                av_log(ctx, AV_LOG_INFO, "histogram_%ddb: %" PRId64 "\n", MAX_DB_FLT - i, vd->histogram[i]);
                sum += vd->histogram[i];
            }
            break;
        case AV_SAMPLE_FMT_S16P:
        case AV_SAMPLE_FMT_S16:
            /* If nb_samples > 1<<34, there is a risk of overflow in the
            multiplication or the sum: shift all histogram values to avoid that.
            The total number of samples must be recomputed to avoid rounding
            errors. */
            shift = av_log2(nb_samples >> 33);
            for (i = 0; i < vd->histogram_size; i++) {
                nb_samples_shift += vd->histogram[i] >> shift;
                power += (i - 0x8000) * (i - 0x8000) * (vd->histogram[i] >> shift);
            }
            if (!nb_samples_shift)
                return;
            power = (power + nb_samples_shift / 2) / nb_samples_shift;
            av_assert0(power <= 0x8000 * 0x8000);
            av_log(ctx, AV_LOG_INFO, "mean_volume: %.1f dB\n", -logdb(ldexp((double)power, -av_log2(HISTOGRAM_SIZE >> 1) * 2), MAX_DB));
            max_volume = 0x8000;
            while (max_volume > 0 && !vd->histogram[0x8000 + max_volume] &&
                                    !vd->histogram[0x8000 - max_volume])
                max_volume--;
            av_log(ctx, AV_LOG_INFO, "max_volume: %.1f dB\n", -logdb(ldexp((double)max_volume * max_volume, -av_log2(HISTOGRAM_SIZE >> 1) * 2), MAX_DB));
            for (i = 0; i < vd->histogram_size; i++)
                histdb[(int)logdb(ldexp((double)(i - 0x8000) * (i - 0x8000), -av_log2(HISTOGRAM_SIZE >> 1) * 2), MAX_DB)] += vd->histogram[i];
            for (i = 0; i <= MAX_DB && !histdb[i]; i++);
            for (; i <= MAX_DB && sum < nb_samples / 1000; i++) {
                av_log(ctx, AV_LOG_INFO, "histogram_%ddb: %"PRId64"\n", -i, histdb[i]);
                sum += histdb[i];
            }
            break;
    }
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    VolDetectContext *vd = ctx->priv;

    switch (outlink->format) {
        case AV_SAMPLE_FMT_S16P:
        case AV_SAMPLE_FMT_S16:
           /*
          * Number of samples at each PCM value.
          * Only used for integer formats.
          * For 16 bit signed PCM there are 65536.
          * histogram[0x8000 + i] is the number of samples at value i.
          * The extra element is there for symmetry.
          */
          vd->histogram_size  = HISTOGRAM_SIZE + 1;
          vd->process_samples = av_sample_fmt_is_planar(outlink->format)
                              ? process_samples_planar_s16 : process_samples_s16;
          break;
        case AV_SAMPLE_FMT_FLT:
        case AV_SAMPLE_FMT_FLTP:
          /*
          * The histogram is used to store the number of samples at each dB
          * instead of the number of samples at each PCM value.
          */
          vd->histogram_size  = HISTOGRAM_SIZE_FLT + 1;
          vd->process_samples = av_sample_fmt_is_planar(outlink->format)
                              ? process_samples_planar_flt : process_samples_flt;
          break;
    }
    vd->sample_fmt = outlink->format;
    vd->histogram = av_calloc(vd->histogram_size, sizeof(uint64_t));
    if (!vd->histogram)
        return AVERROR(ENOMEM);
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    VolDetectContext *vd = ctx->priv;
    print_stats(ctx);
    if (vd->histogram)
        av_freep(&vd->histogram);
}

static const AVFilterPad volumedetect_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad volumedetect_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const AVFilter ff_af_volumedetect = {
    .name          = "volumedetect",
    .description   = NULL_IF_CONFIG_SMALL("Detect audio volume."),
    .priv_size     = sizeof(VolDetectContext),
    .uninit        = uninit,
    .flags         = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(volumedetect_inputs),
    FILTER_OUTPUTS(volumedetect_outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_S16,
                      AV_SAMPLE_FMT_S16P,
                      AV_SAMPLE_FMT_FLT,
                      AV_SAMPLE_FMT_FLTP),
};
