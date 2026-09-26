/*
 * Host test for the DMR demodulator + decoder (main/dmr/), no ESP-IDF needed.
 *
 *   ./dmr_host_test file.dis        raw S16LE 48 kS/s discriminator samples
 *                                   (e.g. DSDcc's samples/dmr_it_8.dis)
 *   ./dmr_host_test file.wav        48 kHz mono 16-bit WAV, same content
 *   ./dmr_host_test -voice out.wav file   (dmr_host_test_voice) also writes
 *                                   the decoded speech, 8 kHz mono
 *   ./dmr_host_test -iq file.cf32   interleaved float32 IQ at 48 kS/s with
 *                                   the channel 12 kHz off centre, exactly as
 *                                   sdr.c hands it over (see dmr_synth.c)
 * Prints every call event, then statistics.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dmr_demod.h"
#include "dmr_proto.h"
#include "dmr_voice.h"

static void print_line(const char *l) { printf("%s\n", l); }

/* -voice out.wav: decoded speech, 8 kHz mono 16 bit (voice build only) */
static FILE *s_wav;
static long s_wav_n;
static void voice_sink(const float *pcm, int n)
{
    for (int i = 0; i < n; i++)
    {
        short v = (short)(pcm[i] * 32767.0f);
        fwrite(&v, 2, 1, s_wav);
    }
    s_wav_n += n;
}
static void wav_header(FILE *f, long n)
{
    unsigned v; unsigned short s;
    fseek(f, 0, SEEK_SET);
    fwrite("RIFF", 1, 4, f); v = (unsigned)(36 + 2 * n); fwrite(&v, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f);
    v = 16; fwrite(&v, 4, 1, f); s = 1; fwrite(&s, 2, 1, f); fwrite(&s, 2, 1, f);
    v = 8000; fwrite(&v, 4, 1, f); v = 16000; fwrite(&v, 4, 1, f); s = 2; fwrite(&s, 2, 1, f); s = 16; fwrite(&s, 2, 1, f);
    fwrite("data", 1, 4, f); v = (unsigned)(2 * n); fwrite(&v, 4, 1, f);
}

int main(int argc, char **argv)
{
    int iq = 0, argi = 1;
    const char *voice_path = NULL;
    FILE *f;
    long sz;
    while (argi < argc && argv[argi][0] == '-')
    {
        if (!strcmp(argv[argi], "-iq")) { iq = 1; argi++; }
        else if (!strcmp(argv[argi], "-voice") && argi + 1 < argc) { voice_path = argv[argi + 1]; argi += 2; }
        else break;
    }
    if (argi >= argc) { fprintf(stderr, "usage: %s [-iq] file\n", argv[0]); return 1; }
    f = fopen(argv[argi], "rb");
    if (!f) { perror(argv[argi]); return 1; }
    fseek(f, 0, SEEK_END); sz = ftell(f); fseek(f, 0, SEEK_SET);

    dmr_demod_init();
    dmr_proto_set_printer(print_line);
    if (voice_path != NULL)
    {
        if (!dmr_voice_available()) { fprintf(stderr, "voice not compiled in (build dmr_host_test_voice)\n"); return 1; }
        s_wav = fopen(voice_path, "wb");
        wav_header(s_wav, 0);
        dmr_voice_set_sink(voice_sink);
    }

    if (iq)
    {
        long n = sz / 8;
        float *buf = malloc(sz), *bi = malloc(n * 4), *bq = malloc(n * 4);
        if (fread(buf, 1, sz, f) != (size_t)sz) return 1;
        for (long k = 0; k < n; k++) { bi[k] = buf[2 * k]; bq[k] = buf[2 * k + 1]; }
        for (long k = 0; k < n; k += 1024) dmr_demod_feed_iq(bi + k, bq + k, (int)(n - k < 1024 ? n - k : 1024));
    }
    else
    {
        long skip = (strstr(argv[argi], ".wav") != NULL) ? 44 : 0, n = (sz - skip) / 2;
        short *s = malloc(sz);
        float *x = malloc(n * 4);
        fseek(f, skip, SEEK_SET);
        if (fread(s, 2, n, f) != (size_t)n) return 1;
        for (long k = 0; k < n; k++) x[k] = s[k];
        for (long k = 0; k < n; k += 1024) dmr_demod_feed_disc(x + k, (int)(n - k < 1024 ? n - k : 1024));
    }
    fclose(f);
    if (s_wav != NULL)
    {
        wav_header(s_wav, s_wav_n);
        fclose(s_wav);
        printf("voice: %.2f s decoded to %s\n", s_wav_n / 8000.0, voice_path);
    }

    {
        dmr_proto_stats_t st;
        dmr_demod_status_t ds;
        dmr_proto_get_stats(&st);
        dmr_demod_get_status(&ds);
        printf("\nbursts %u (FEC-valid %u)  syncs %u  voice-LC hdr %u  terminators %u  emb LC ok %u bad %u  csbk %u  idle %u\n",
               st.frames, st.frames_valid, ds.syncs, st.lc_headers, st.terminators, st.emb_lc_ok, st.emb_lc_bad, st.csbk, st.idle);
        printf("levels: outer %.1f  dc %.1f  inverted %d  (%s)\n", ds.level, ds.dc, ds.inverted, ds.bs ? "BS" : ds.ms ? "MS" : "direct/none");
        for (int s = 0; s < 2; s++)
        {
            dmr_slot_info_t si;
            dmr_proto_get_slot(s, &si);
            printf("slot %d: cc %d  ids %s %s %u <- %u  alias \"%s\"\n", s + 1, si.cc, si.ids_valid ? "yes" : "no",
                   si.group ? "TG" : "ID", si.dst, si.src, si.alias);
        }
    }
    return 0;
}
