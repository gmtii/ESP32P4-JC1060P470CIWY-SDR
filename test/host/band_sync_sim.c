/*
 * Host simulation of the band-sync loop (ft8_band_sync.c) driving the real
 * ported FT8 pipeline. A continuous "band" is built by concatenating 12 kHz
 * corpus WAVs (each one is one 15 s slot). The local clock is off by an
 * initial amount; the simulation reproduces ft8_app.c's scheduling exactly:
 *  - a capture starts at a local 15 s boundary and collects 93 blocks,
 *  - at the next boundary the next capture starts and the previous one is
 *    decoded; the loop's correction is applied to the clock,
 *  - corrections <= FT8_BSYNC_RESTART_MS are absorbed (the capture already
 *    running continues on the old grid, the loop ignores its timing);
 *    larger ones drop that capture and re-arm at the next new boundary.
 *
 * usage: ./band_sync_sim initial_offset_s clock_set(0|1) file1.wav file2.wav ...
 *   initial_offset_s: local clock minus band time (e.g. 6.3 = 6.3 s ahead)
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include "ft8_ram.h"
#include "ft8_decimator.h"
#include "ft8_waterfall_adapter.h"
#include "ft8_decoder.h"
#include "ft8_band_sync.h"

#define FS 12000
#define SLOT_S 15.0

void ft8_time_get_slot_start_utc(struct tm *out) { memset(out, 0, sizeof(*out)); }

static int16_t *band;
static long band_len;

static int16_t *read_wav(const char *path, int *n_out)
{
    FILE *f = fopen(path, "rb");
    unsigned char h[44];
    int16_t *d;
    long sz;
    if (!f) return NULL;
    fseek(f, 0, SEEK_END); sz = ftell(f); fseek(f, 0, SEEK_SET);
    if (fread(h, 1, 44, f) != 44) { fclose(f); return NULL; }
    d = malloc(sz);
    *n_out = (int)fread(d, 2, (sz - 44) / 2, f);
    fclose(f);
    return d;
}

/* decode a capture that starts at band time t0 (seconds) */
static void capture_and_decode(double t0)
{
    long start = (long)floor(t0 * FS);
    float blk[256];
    ft8_decimator_reset();
    ft8_waterfall_reset();
    while (!ft8_waterfall_is_full())
    {
        for (int k = 0; k < 256; k++)
        {
            long idx = ((start++ % band_len) + band_len) % band_len;
            blk[k] = (float)band[idx];
        }
        ft8_decimator_feed(blk, 256);
    }
    ft8_waterfall_snapshot_mag();
    ft8_decoder_process_slot();
    { ft8_decoded_msg_t m; while (ft8_decoder_get_message(&m)) { } }
}

int main(int argc, char **argv)
{
    double a = atof(argv[1]); /* local = band + a */
    int clock_set = atoi(argv[2]);
    int nfiles = argc - 3;
    band_len = (long)(nfiles * SLOT_S * FS);
    band = calloc(band_len, 2);
    for (int i = 0; i < nfiles; i++)
    {
        int n; int16_t *d = read_wav(argv[3 + i], &n);
        if (!d) { fprintf(stderr, "bad wav %s\n", argv[3 + i]); return 1; }
        if (n > (int)(SLOT_S * FS)) n = (int)(SLOT_S * FS);
        memcpy(&band[(long)(i * SLOT_S * FS)], d, n * 2);
        free(d);
    }
    ft8_ram_init();
    ft8_decoder_init();
    ft8_bsync_reset(clock_set);

    /* first capture starts at the first local boundary after band time 0 */
    double t_cap = ceil((0.0 + a) / SLOT_S) * SLOT_S - a; /* band time of that boundary */
    int restarts = 0;
    double shift_during = 0.0; /* clock jump applied while the current capture was running */
    for (int slot = 0; slot < 28; slot++)
    {
        const float *dt;
        int ndt;
        float err;
        int32_t delta;
        ft8_bsync_input_t in;

        capture_and_decode(t_cap);
        ndt = ft8_decoder_get_last_timing(&dt);
        in.n_dt = ndt; in.dt_s = dt; in.top_score = ft8_decoder_get_last_top_score();
        delta = ft8_bsync_slot(&in);

        /* the boundary that ended this capture: 15 local seconds after it
         * started, minus any clock jump applied while it was running */
        double t_now = t_cap + SLOT_S - shift_during;
        double next_cap = t_now;      /* capture already running since t_now (old grid) */
        a += delta / 1000.0;
        shift_during = delta / 1000.0; /* happens during the capture that starts at t_now */
        if (delta != 0 && abs(delta) > FT8_BSYNC_RESTART_MS)
        {
            /* drop the running capture, re-arm at the next new-grid boundary */
            next_cap = ceil((t_now + 0.001 + a) / SLOT_S) * SLOT_S - a;
            ft8_bsync_capture_restarted();
            restarts++;
            shift_during = 0.0;
        }
        else if (delta != 0)
        {
            /* running capture continues; the one after it follows the new grid */
        }

        /* band-referenced phase error of the local grid: capture start vs the
         * true slot boundary (band time multiple of 15 s) */
        double ph = fmod(t_cap, SLOT_S); if (ph > 7.5) ph -= 15.0;
        {
            char med[16] = "   --  ";
            if (ndt && ft8_bsync_get_last_error(&err)) snprintf(med, sizeof(med), "%+.3f", err);
            printf("slot %2d  grid@%+6.2fs  dec=%2d top=%2d  medDT=%7s -> %-16s corr=%+6ld ms\n",
                   slot, ph, ndt, in.top_score, med,
                   ft8_bsync_state_name(ft8_bsync_get_state()), (long)delta);
        }
        t_cap = next_cap;
    }
    return 0;
}
