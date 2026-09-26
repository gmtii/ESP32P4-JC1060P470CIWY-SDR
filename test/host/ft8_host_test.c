/*
 * Host-side regression test for the ported FT8 pipeline (no ESP-IDF needed).
 * Feeds a 12 kHz mono 16-bit WAV (e.g. ft8_lib's test/wav corpus) through the
 * exact same code the ESP32-P4 runs - ft8_decimator (AGC + 12k->3200 resampler)
 * -> ft8_waterfall_adapter (1024-pt FFT, mag[] encode) -> snapshot ->
 * ft8_decoder - and prints the decoded lines.
 *
 * Build (from this folder, ft8_lib checked out next to it or at FT8LIB):
 *   make FT8LIB=../../components/ft8_lib
 * Run:
 *   ./ft8_host_test file.wav [gain]
 * gain (default 1.0) scales the float input before the pipeline, to check
 * that the FT8 AGC makes decoding independent of the input level (the P4 feeds
 * +-1.0 floats scaled by 32768, like ft8_app.c does).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include "ft8_ram.h"
#include "ft8_decimator.h"
#include "ft8_waterfall_adapter.h"
#include "ft8_decoder.h"

/* ft8_time.h stub: the decoder only needs a slot timestamp. */
void ft8_time_get_slot_start_utc(struct tm *out)
{
    memset(out, 0, sizeof(*out));
}

static int16_t *read_wav(const char *path, int *n_out, int *rate_out)
{
    FILE *f = fopen(path, "rb");
    unsigned char hdr[12];
    int16_t *data = NULL;
    if (!f) return NULL;
    if (fread(hdr, 1, 12, f) != 12) { fclose(f); return NULL; }
    for (;;)
    {
        unsigned char ch[8];
        uint32_t sz;
        if (fread(ch, 1, 8, f) != 8) break;
        sz = ch[4] | (ch[5] << 8) | (ch[6] << 16) | ((uint32_t)ch[7] << 24);
        if (memcmp(ch, "fmt ", 4) == 0)
        {
            unsigned char fmt[64];
            if (fread(fmt, 1, sz, f) != sz) break;
            *rate_out = fmt[4] | (fmt[5] << 8) | (fmt[6] << 16) | (fmt[7] << 24);
        }
        else if (memcmp(ch, "data", 4) == 0)
        {
            data = malloc(sz);
            if (fread(data, 1, sz, f) != sz) { free(data); data = NULL; break; }
            *n_out = (int)(sz / 2);
            break;
        }
        else
        {
            fseek(f, sz, SEEK_CUR);
        }
    }
    fclose(f);
    return data;
}

int main(int argc, char **argv)
{
    int n = 0, rate = 0, i;
    float gain = (argc > 2) ? (float)atof(argv[2]) : 1.0f;
    int16_t *pcm;
    float blk[256];
    ft8_decoded_msg_t msg;
    float dmin, dmax;

    if (argc < 2) { fprintf(stderr, "usage: %s file.wav [gain]\n", argv[0]); return 1; }
    pcm = read_wav(argv[1], &n, &rate);
    if (!pcm || rate != 12000) { fprintf(stderr, "need a 12 kHz 16-bit mono WAV\n"); return 1; }
    if (!ft8_ram_init()) { fprintf(stderr, "alloc failed\n"); return 1; }

    ft8_decoder_init();
    ft8_decimator_reset();
    ft8_waterfall_reset();

    /* Same framing as the P4: 256-sample blocks (one sdrTask loop at 12 kHz),
     * float in +-1.0 scaled by 32768 (ft8_app.c's FT8_INPUT_SCALE). */
    for (i = 0; i + 256 <= n && !ft8_waterfall_is_full(); i += 256)
    {
        for (int k = 0; k < 256; k++)
            blk[k] = ((float)pcm[i + k] / 32768.0f) * gain * 32768.0f;
        ft8_decimator_feed(blk, 256);
    }
    /* pad with silence if the file is a bit short */
    while (!ft8_waterfall_is_full())
    {
        memset(blk, 0, sizeof(blk));
        ft8_decimator_feed(blk, 256);
    }

    ft8_waterfall_snapshot_mag();
    ft8_waterfall_get_db_range(&dmin, &dmax);
    ft8_decoder_process_slot();

    printf("# top=%d cand=%d dec=%d dB[%.0f..%.0f]\n", ft8_decoder_get_last_top_score(), ft8_decoder_get_last_num_candidates(),
           ft8_decoder_get_last_num_decoded(), dmin, dmax);
    {
        const float *dt;
        int ndt = ft8_decoder_get_last_timing(&dt), k = 0;
        while (ft8_decoder_get_message(&msg))
        {
            /* raw DT (3 decimals) after a tab, for calibrate_dt.py */
            if (k < ndt) printf("%s\t%+.3f\n", msg.line, dt[k]);
            else printf("%s\n", msg.line);
            k++;
        }
    }
    free(pcm);
    return 0;
}
