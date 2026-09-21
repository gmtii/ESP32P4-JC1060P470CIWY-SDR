/*
 * rtl_dsp.h - CU8 (RTL-SDR) -> 48 kHz I/Q conversion chain.
 *
 * Pure C, no ESP-IDF dependencies, so it can be unit-tested on the host.
 *
 *   CU8 @ 960 kSps
 *     -> CIC (N=4, R=5)                      -> 192 kSps
 *     -> variable-delay cubic Lagrange       (absorbs clock drift between the
 *        resampler, step = 1 +/- few ppm      dongle crystal and the codec clock)
 *     -> 96-tap Kaiser FIR, decimate by 4    -> 48 kSps, int16 interleaved I/Q
 *
 * The drift resampler runs at 192 kSps on purpose: at that rate the wanted
 * signal (a few tens of kHz at most) sits at < 0.125 fs, where a cubic
 * interpolator is flat to < 0.1 dB. Doing it at 48 kSps would cost ~1 dB
 * at the 12 kHz IF used by sdr.c.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RTL_DSP_IN_RATE    960000u                          /* dongle sample rate            */
#define RTL_DSP_CIC_R      5u                               /* CIC decimation                */
#define RTL_DSP_MID_RATE   (RTL_DSP_IN_RATE / RTL_DSP_CIC_R) /* 192 kSps                     */
#define RTL_DSP_FIR_R      4u                               /* FIR decimation                */
#define RTL_DSP_OUT_RATE   (RTL_DSP_MID_RATE / RTL_DSP_FIR_R) /* 48 kSps                     */
#define RTL_DSP_FIR_TAPS   96

#define RTL_DSP_STEP_MIN   0.99f
#define RTL_DSP_STEP_MAX   1.01f

/* Upper bound of output frames produced from n_bytes of CU8 input. */
#define RTL_DSP_MAX_FRAMES(n_bytes) \
    ((size_t)(n_bytes) / 2u / (RTL_DSP_CIC_R * RTL_DSP_FIR_R) * 102u / 100u + 4u)

typedef struct {
    /* CIC state, 4 integrators + 4 combs per channel (unsigned: defined wrap-around) */
    uint32_t integ_i[4], integ_q[4];
    uint32_t comb_i[4], comb_q[4];
    uint32_t cic_cnt;

    /* Cubic Lagrange interpolator (192 kSps) */
    float hist_i[4], hist_q[4];
    uint32_t hist_fill;
    float mu;
    float step;

    /* Decimating FIR (192 -> 48 kSps); doubled delay line avoids modulo in the MAC loop */
    float dl_i[2 * RTL_DSP_FIR_TAPS], dl_q[2 * RTL_DSP_FIR_TAPS];
    uint32_t dl_pos;
    uint32_t fir_cnt;
    float coeff[RTL_DSP_FIR_TAPS];

    bool conjugate;         /* negate Q on output (flip spectrum orientation) */
    uint32_t out_dropped;   /* frames lost because max_out was too small      */
} rtl_dsp_t;

/* Reset all state and (re)design the FIR. conjugate: see above. */
void rtl_dsp_init(rtl_dsp_t *d, bool conjugate);

/* Resampling step in input samples per output sample at 192 kSps (1.0 = nominal). Clamped. */
void rtl_dsp_set_step(rtl_dsp_t *d, float step);

/*
 * Convert n_bytes of interleaved CU8 (I0,Q0,I1,Q1,...) to 48 kSps frames.
 * out receives interleaved int16 I,Q (2 * frames entries). Returns frames written.
 * Size out with RTL_DSP_MAX_FRAMES(n_bytes).
 */
size_t rtl_dsp_process(rtl_dsp_t *d, const uint8_t *cu8, size_t n_bytes,
                       int16_t *out, size_t max_out_frames);

/* ------------------------------------------------------------------------- */
/* Clock-drift controller                                                    */
/* ------------------------------------------------------------------------- */

/*
 * The FIFO between the USB side and the codec side is an integrator, so a
 * proportional controller on its (smoothed) fill level is enough: it removes
 * the rate error and leaves a small, constant level offset proportional to the
 * ppm difference. No integral term, no hunting.
 *
 * The level must be sampled at the consumer's pop instants and corrected for
 * the fact that USB data arrives in 8.5 ms lumps (rtl_level_estimate). Sampling
 * the raw fill on every USB block instead makes the estimate beat against the
 * pop cadence (960000/8192 Hz vs 48000/1024 Hz are exactly 5:2), and that beat
 * shows up as a slow, large ripple of the resampling step.
 */
#define RTL_RATE_PRIME_FRAMES  3584u     /* the consumer starts once this many frames are queued (~75 ms) */
#define RTL_RATE_TARGET_FRAMES 2560u     /* level the controller holds (~53 ms). The first TX write
                                            lands in an empty I2S DMA ring and returns at once, so the
                                            consumer takes ~1 extra block up front, which brings the
                                            FIFO from PRIME to about TARGET without any resampling. */
#define RTL_RATE_KP            3.0e-6f   /* step change per frame of level error */
#define RTL_RATE_AVG_ALPHA     (1.0f / 64.0f)   /* per consumer block (~21 ms)   */
#define RTL_RATE_STEP_LIMIT    2.0e-3f   /* +/- 2000 ppm                         */

typedef struct {
    float fill_avg;
    float step;
} rtl_rate_ctl_t;

void  rtl_rate_ctl_reset(rtl_rate_ctl_t *c);

/*
 * FIFO level in frames, as if the USB data had arrived continuously.
 *   fifo_frames : produced - consumed at the moment of the last USB block
 *   since_us    : time elapsed since that block was delivered
 */
static inline float rtl_level_estimate(uint32_t fifo_frames, int64_t since_us)
{
    float since = (float)since_us * 1e-6f * (float)RTL_DSP_OUT_RATE;
    if (since < 0.0f) since = 0.0f;
    if (since > 1024.0f) since = 1024.0f;   /* a block is ~410 frames; more means the dongle stalled */
    return (float)fifo_frames + since;
}

/* Call once per consumer block, before popping. primed=false holds the step at 1.0. */
float rtl_rate_ctl_update(rtl_rate_ctl_t *c, float level_frames, bool primed);

#ifdef __cplusplus
}
#endif
