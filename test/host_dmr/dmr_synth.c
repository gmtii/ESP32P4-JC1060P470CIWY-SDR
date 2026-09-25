/*
 * DMR test-signal synthesizer (host only). Builds complete bursts with the
 * firmware's own encoders (dmr_fec.c) and modulates them as 4FSK/FM:
 *
 *   ./dmr_synth bs|ms out_prefix [snr_db] [freq_err_hz] [normal]
 *
 * bs: a repeater (BS sourced) stream - TS1 idle, TS2 a group call from
 *     2140123 to TG 214 with talker alias "EA8DGL", voice LC headers, 4
 *     voice superframes (embedded LC: IDs, then talker alias header and
 *     block 1), terminators; a CSBK on TS1 at the end.
 * ms: the same call as an MS sourced (simplex) stream, bursts every 60 ms.
 *
 * Environment:
 *   DMR_SYNTH_SF=n      voice superframes in the call (default 4)
 *   DMR_SYNTH_AMBE=f    replay real voice bursts (132 dibits each, as written
 *                       by a -DDMR_VOICE_BURST_DUMP decode) instead of random
 *                       AMBE bits - lets speech quality be judged at low SNR
 *
 * Writes:
 *   out_prefix.cf32  interleaved float32 IQ at 48 kS/s as the ESP32-P4 board's
 *                    rtl_source delivers it: CONJUGATED (channel 12 kHz below
 *                    centre, FM deviation and tuning error negated - confirmed
 *                    on the radio), + AWGN. "normal" gives the non-conjugated
 *                    orientation instead (tests the automatic polarity flip).
 *   out_prefix.wav   clean discriminator audio (48 kHz, 16 bit) for an
 *                    independent decoder (dsd-fme -fs -i out_prefix.wav)
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dmr_fec.h"

#define FS 48000
#define SPS 10
#define MAXSYM 400000

static const uint64_t SYNC_BS_VOICE = 0x755FD7DF75F7ull, SYNC_BS_DATA = 0xDFF57D75DF5Dull;
static const uint64_t SYNC_MS_VOICE = 0x7F7D5DD57DFDull, SYNC_MS_DATA = 0xD5D7F77FD757ull;

static uint8_t dibits[MAXSYM];
static uint8_t *g_real_ambe;
static int g_real_n, g_real_pos;
static uint8_t keyed[MAXSYM]; /* 0 = transmitter off (MS gaps) */
static int nsym;
static int ms_mode;

static void put_bits(const uint8_t *bits, int n, int on)
{
    for (int i = 0; i < n; i += 2)
    {
        keyed[nsym] = (uint8_t)on;
        dibits[nsym++] = (uint8_t)((bits[i] << 1) | bits[i + 1]);
    }
}

static void u64_bits(uint64_t v, int n, uint8_t *b)
{
    for (int i = 0; i < n; i++) b[i] = (uint8_t)((v >> (n - 1 - i)) & 1u);
}

static void cach(int slot, int on)
{
    uint8_t c[24] = {0}, tact[7];
    static const int pos[7] = {0, 4, 8, 12, 14, 18, 22};
    uint32_t cw = dmr_hamming_7_4_encode((1u << 3) | ((uint32_t)slot << 2)); /* AT=1, TC=slot, LCSS=0 */
    dmr_u32_to_bits(cw, 7, tact);
    for (int i = 0; i < 7; i++) c[pos[i]] = tact[i];
    put_bits(c, 24, on);
}

/* burst with payload halves (108 bits each) and a 48-bit centre field */
static void burst(int slot, const uint8_t *first, const uint8_t *centre, const uint8_t *second)
{
    static const uint8_t z[288] = {0};
    if (!ms_mode) cach(slot, 1);
    else put_bits(z, 24, 0); /* MS: the 12-symbol CACH position is silent */
    put_bits(first, 108, 1);
    put_bits(centre, 48, 1);
    put_bits(second, 108, 1);
    if (ms_mode) put_bits(z, 288, 0); /* ...and so is the other timeslot (60 ms period) */
}

static void data_burst(int slot, int cc, int dt, const uint8_t info96[96])
{
    uint8_t coded[196], st[20], a[108], b[108], c[48];
    dmr_bptc196_encode(info96, coded);
    dmr_u32_to_bits(dmr_golay_20_8_encode((uint32_t)((cc << 4) | dt)), 20, st);
    memcpy(a, coded, 98);
    memcpy(&a[98], st, 10);
    memcpy(b, &st[10], 10);
    memcpy(&b[10], &coded[98], 98);
    u64_bits(ms_mode ? SYNC_MS_DATA : SYNC_BS_DATA, 48, c);
    burst(slot, a, c, b);
}

static void lc_bytes_to_bits(const uint8_t *by, int n, uint8_t *bits)
{
    for (int i = 0; i < n; i++) dmr_u32_to_bits(by[i], 8, &bits[8 * i]);
}

static void full_lc(int slot, int cc, int dt, uint32_t mask, uint32_t dst, uint32_t src)
{
    uint8_t by[12] = {0x00, 0x00, 0x00, (uint8_t)(dst >> 16), (uint8_t)(dst >> 8), (uint8_t)dst,
                      (uint8_t)(src >> 16), (uint8_t)(src >> 8), (uint8_t)src};
    uint8_t bits[96];
    dmr_rs_12_9_parity(by, mask, &by[9]);
    lc_bytes_to_bits(by, 12, bits);
    data_burst(slot, cc, dt, bits);
}

/* one voice superframe (bursts A..F) carrying one embedded LC */
static void voice_superframe(int slot, int cc, const uint8_t lc72[72], int idle_other_slot)
{
    uint8_t emb128[128];
    dmr_emb_lc_encode(lc72, emb128);
    for (int k = 0; k < 6; k++)
    {
        uint8_t a[108], b[108], c[48];
        for (int i = 0; i < 108; i++) { a[i] = rand() & 1; b[i] = rand() & 1; } /* AMBE placeholder */
        if (g_real_ambe != NULL && g_real_n > 0)
        {
            /* DMR_SYNTH_AMBE=file: real voice bursts (132 dibits each, from a
             * DMR_VOICE_BURST_DUMP decode) - replay their AMBE payload */
            const uint8_t *vb = &g_real_ambe[132 * (g_real_pos++ % g_real_n)];
            for (int i = 0; i < 54; i++) { a[2 * i] = (vb[i] >> 1) & 1; a[2 * i + 1] = vb[i] & 1; }
            for (int i = 0; i < 54; i++) { b[2 * i] = (vb[78 + i] >> 1) & 1; b[2 * i + 1] = vb[78 + i] & 1; }
        }
        if (k == 0)
        {
            u64_bits(ms_mode ? SYNC_MS_VOICE : SYNC_BS_VOICE, 48, c);
        }
        else
        {
            static const int lcss_of[6] = {0, 1, 3, 3, 2, 0};
            uint8_t emb[16];
            uint32_t cw = dmr_qr_16_7_encode((uint32_t)((cc << 3) | lcss_of[k]));
            dmr_u32_to_bits(cw, 16, emb);
            memcpy(c, emb, 8);
            if (k <= 4) memcpy(&c[8], &emb128[(k - 1) * 32], 32);
            else memset(&c[8], 0, 32);
            memcpy(&c[40], &emb[8], 8);
        }
        burst(slot, a, c, b);
        if (!ms_mode && idle_other_slot)
        {
            uint8_t idle[96];
            for (int i = 0; i < 96; i++) idle[i] = rand() & 1;
            data_burst(1 - slot, cc, 9, idle);
        }
    }
}

static void emb_lc_bytes(uint8_t lc[72], const uint8_t by[9]) { lc_bytes_to_bits(by, 9, lc); }

int main(int argc, char **argv)
{
    const int cc = 1, slot = 1; /* TS2 */
    const uint32_t dst = 214, src = 2140123;
    float snr_db = argc > 3 ? (float)atof(argv[3]) : 30.0f;
    float ferr = argc > 4 ? (float)atof(argv[4]) : 0.0f;
    int normal = argc > 5 && !strcmp(argv[5], "normal");
    char path[256];

    if (argc < 3) { fprintf(stderr, "usage: %s bs|ms prefix [snr_db] [freq_err_hz]\n", argv[0]); return 1; }
    ms_mode = !strcmp(argv[1], "ms");
    srand(1234);
    if (getenv("DMR_SYNTH_AMBE"))
    {
        FILE *af = fopen(getenv("DMR_SYNTH_AMBE"), "rb");
        if (af)
        {
            fseek(af, 0, SEEK_END);
            long sz = ftell(af);
            fseek(af, 0, SEEK_SET);
            g_real_ambe = malloc((size_t)sz);
            g_real_n = (int)(fread(g_real_ambe, 1, (size_t)sz, af) / 132);
            fclose(af);
        }
    }

    /* preamble: idle bursts so the receiver can lock (BS) / silence (MS) */
    for (int k = 0; k < 8; k++)
    {
        uint8_t idle[96];
        for (int i = 0; i < 96; i++) idle[i] = rand() & 1;
        if (!ms_mode) data_burst(k & 1, cc, 9, idle);
    }
    for (int k = 0; k < 3; k++)
    {
        full_lc(slot, cc, 1, DMR_RS_MASK_VOICE_HEADER, dst, src);
        if (!ms_mode) { uint8_t idle[96] = {0}; data_burst(1 - slot, cc, 9, idle); }
    }
    {
        uint8_t lc[72];
        const uint8_t grp[9] = {0x00, 0x00, 0x00, (uint8_t)(dst >> 16), (uint8_t)(dst >> 8), (uint8_t)dst,
                                (uint8_t)(src >> 16), (uint8_t)(src >> 8), (uint8_t)src};
        /* talker alias "EA8DGL", 7-bit format, 6 chars: header carries 49 bits = 7 chars */
        const char *ta = "EA8DGL ";
        uint8_t tabits[49 + 56] = {0}, hdr[72] = {0};
        for (int i = 0; i < 7; i++) dmr_u32_to_bits((uint32_t)(unsigned char)ta[i], 7, &tabits[7 * i]);
        dmr_u32_to_bits(0x04, 8, hdr);    /* FLCO 4: TA header, FID 0 */
        dmr_u32_to_bits(0x00, 2, &hdr[16]); /* format: 7 bit */
        dmr_u32_to_bits(6, 5, &hdr[18]);  /* 6 characters */
        memcpy(&hdr[23], tabits, 49);

        /* DMR_SYNTH_SF=n: a longer call (default 4 superframes) */
        int nsf = getenv("DMR_SYNTH_SF") ? atoi(getenv("DMR_SYNTH_SF")) : 4;
        emb_lc_bytes(lc, grp);
        voice_superframe(slot, cc, lc, 1);
        voice_superframe(slot, cc, hdr, 1);
        for (int s = 2; s < nsf; s++) voice_superframe(slot, cc, lc, 1);
    }
    for (int k = 0; k < 2; k++)
    {
        full_lc(slot, cc, 2, DMR_RS_MASK_TERMINATOR, dst, src);
        if (!ms_mode) { uint8_t idle[96] = {0}; data_burst(1 - slot, cc, 9, idle); }
    }
    if (!ms_mode)
    {
        uint8_t csbk[96] = {0};
        uint16_t crc;
        dmr_u32_to_bits(0x3D, 8, csbk); /* a harmless CSBK opcode */
        crc = (uint16_t)(dmr_crc_ccitt(csbk, 80) ^ DMR_CRC_MASK_CSBK);
        dmr_u32_to_bits(crc, 16, &csbk[80]);
        data_burst(0, cc, 3, csbk);
        for (int k = 0; k < 6; k++) { uint8_t idle[96] = {0}; data_burst(k & 1, cc, 9, idle); }
    }

    /* ---- 4FSK: RRC-shaped deviation, FM to IQ ---- */
    {
        const float pi = 3.14159265358979f, a = 0.2f;
        const int taps = 81, n = nsym * SPS + taps;
        float h[81], hs = 0.0f;
        float *dev = calloc((size_t)n, sizeof(float));
        float *env = calloc((size_t)n, sizeof(float));
        FILE *fiq, *fw;
        double ph = 0.0;
        for (int i = 0; i < taps; i++)
        {
            float t = (i - (taps - 1) / 2.0f) / SPS, v;
            if (fabsf(t) < 1e-6f) v = 1 - a + 4 * a / pi;
            else if (fabsf(fabsf(t) - 1 / (4 * a)) < 1e-6f) v = a / sqrtf(2) * ((1 + 2 / pi) * sinf(pi / (4 * a)) + (1 - 2 / pi) * cosf(pi / (4 * a)));
            else v = (sinf(pi * t * (1 - a)) + 4 * a * t * cosf(pi * t * (1 + a))) / (pi * t * (1 - (4 * a * t) * (4 * a * t)));
            h[i] = v; hs += v;
        }
        /* impulses at symbol centres, amplitude scaled so the RRC-shaped
         * deviation reaches the nominal +-648 / +-1944 Hz */
        for (int s = 0; s < nsym; s++)
        {
            static const float lev[4] = {648.0f, 1944.0f, -648.0f, -1944.0f}; /* dibit 00,01,10,11 */
            for (int i = 0; i < taps; i++) dev[s * SPS + i] += lev[dibits[s]] * SPS * h[i] / hs;
            for (int i = 0; i < SPS; i++) env[s * SPS + taps / 2 + i] = keyed[s] ? 1.0f : 0.0f;
        }
        snprintf(path, sizeof(path), "%s.cf32", argv[2]);
        fiq = fopen(path, "wb");
        snprintf(path, sizeof(path), "%s.wav", argv[2]);
        fw = fopen(path, "wb");
        {
            uint32_t bytes = (uint32_t)n * 2, v;
            fwrite("RIFF", 1, 4, fw); v = 36 + bytes; fwrite(&v, 4, 1, fw); fwrite("WAVEfmt ", 1, 8, fw);
            v = 16; fwrite(&v, 4, 1, fw); { uint16_t f = 1, ch = 1; fwrite(&f, 2, 1, fw); fwrite(&ch, 2, 1, fw); }
            v = FS; fwrite(&v, 4, 1, fw); v = FS * 2; fwrite(&v, 4, 1, fw); { uint16_t ba = 2, bps = 16; fwrite(&ba, 2, 1, fw); fwrite(&bps, 2, 1, fw); }
            fwrite("data", 1, 4, fw); fwrite(&bytes, 4, 1, fw);
        }
        {
            /* noise: SNR measured in a 12.5 kHz channel, carrier power 1 */
            float sigma = sqrtf(powf(10.0f, -snr_db / 10.0f) * (FS / 12500.0f) / 2.0f);
            for (int k = 0; k < n; k++)
            {
                float fi, fq, u1, u2, r;
                int16_t pcm = (int16_t)(dev[k] * 8.0f); /* ~ +-15500 at full deviation */
                fwrite(&pcm, 2, 1, fw);
                /* channel at -12 kHz; the board conjugates: deviation and error negated */
                ph += 2.0 * M_PI * ((normal ? 1.0 : -1.0) * (dev[k] + ferr) - 12000.0) / FS;
                fi = env[k] * (float)cos(ph);
                fq = env[k] * (float)sin(ph);
                u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f); u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
                r = sigma * sqrtf(-2.0f * logf(u1));
                fi += r * cosf(2 * pi * u2);
                fq += r * sinf(2 * pi * u2);
                fwrite(&fi, 4, 1, fiq);
                fwrite(&fq, 4, 1, fiq);
            }
        }
        fclose(fiq);
        fclose(fw);
        printf("%s: %d symbols, %.2f s, SNR %.1f dB, frequency error %+.0f Hz\n", ms_mode ? "MS" : "BS", nsym, n / (float)FS, snr_db, ferr);
    }
    return 0;
}
