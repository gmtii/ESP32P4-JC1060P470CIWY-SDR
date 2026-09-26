/*
 * AIS test-signal synthesizer (host only): encodes known messages, frames
 * them (training + flags + bit stuffing + CRC-16/X.25), NRZI-encodes and
 * GMSK-modulates them (BT 0.4, 9600 baud, +-2400 Hz) on both AIS channels.
 *
 *   ./ais_synth out.cf32 [rate] [snr_db] [freq_err_hz] [normal]
 *
 * rate: output sample rate (default 192000 = the firmware's wide path).
 * Default orientation is the ESP32-P4 board's (conjugated I/Q: channel A at
 * +25 kHz); "normal" gives the standard one (A at -25 kHz) for other decoders.
 * Channel A and B carry different messages, partly overlapping in time.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ais_demod.h" /* ais_crc16 */

/* ---- message bit writer ---- */
typedef struct { uint8_t b[128]; int n; } msg_t;
static void put(msg_t *m, uint32_t v, int nb) { for (int i = nb - 1; i >= 0; i--) { if ((v >> i) & 1u) m->b[m->n / 8] |= (uint8_t)(0x80 >> (m->n % 8)); m->n++; } }
static void puts6(msg_t *m, const char *s, int nch)
{
    for (int i = 0; i < nch; i++)
    {
        int ch = (i < (int)strlen(s)) ? (unsigned char)s[i] : '@';
        int v = ch >= 64 ? ch - 64 : ch; /* AIS 6-bit ASCII */
        put(m, (uint32_t)(v & 63), 6);
    }
}
static uint32_t lonlat(double deg, int nb) { int32_t v = (int32_t)lround(deg * 600000.0); return (uint32_t)v & ((nb == 32) ? 0xFFFFFFFFu : ((1u << nb) - 1u)); }

static msg_t type1(void)
{ msg_t m = {{0}, 0}; put(&m,1,6); put(&m,0,2); put(&m,224123456,30); put(&m,0,4); put(&m,0,8); put(&m,123,10); put(&m,1,1);
  put(&m,lonlat(-16.2500,28),28); put(&m,lonlat(28.4700,27),27); put(&m,456,12); put(&m,46,9); put(&m,30,6); put(&m,0,2); put(&m,0,3); put(&m,0,1); put(&m,0,19); return m; }
static msg_t type5(void)
{ msg_t m = {{0}, 0}; put(&m,5,6); put(&m,0,2); put(&m,224123456,30); put(&m,0,2); put(&m,9123456,30); puts6(&m,"EA1234",7); puts6(&m,"NAVE TEST ONE",20);
  put(&m,70,8); put(&m,100,9); put(&m,20,9); put(&m,8,6); put(&m,8,6); put(&m,1,4); put(&m,9,4); put(&m,26,5); put(&m,18,5); put(&m,0,6); put(&m,65,8);
  puts6(&m,"SANTA CRUZ",20); put(&m,0,1); put(&m,0,1); return m; }
static msg_t type18(void)
{ msg_t m = {{0}, 0}; put(&m,18,6); put(&m,0,2); put(&m,224999888,30); put(&m,0,8); put(&m,50,10); put(&m,0,1);
  put(&m,lonlat(-16.3000,28),28); put(&m,lonlat(28.5000,27),27); put(&m,1800,12); put(&m,511,9); put(&m,15,6); put(&m,0,2); put(&m,1,1); put(&m,0,1); put(&m,1,1); put(&m,1,1); put(&m,1,1); put(&m,0,1); put(&m,0,1); put(&m,0,20); return m; }
static msg_t type24a(void)
{ msg_t m = {{0}, 0}; put(&m,24,6); put(&m,0,2); put(&m,224999888,30); put(&m,0,2); puts6(&m,"VELERO",20); return m; }
static msg_t type4(void)
{ msg_t m = {{0}, 0}; put(&m,4,6); put(&m,0,2); put(&m,2241234,30); put(&m,2026,14); put(&m,9,4); put(&m,26,5); put(&m,10,5); put(&m,20,6); put(&m,30,6); put(&m,1,1);
  put(&m,lonlat(-16.2400,28),28); put(&m,lonlat(28.4800,27),27); put(&m,7,4); put(&m,0,10); put(&m,0,1); put(&m,0,19); return m; }
static msg_t type21(void)
{ msg_t m = {{0}, 0}; put(&m,21,6); put(&m,0,2); put(&m,992241001,30); put(&m,6,5); puts6(&m,"FARO ANAGA",20); put(&m,1,1);
  put(&m,lonlat(-16.1300,28),28); put(&m,lonlat(28.5700,27),27); put(&m,0,9); put(&m,0,9); put(&m,0,6); put(&m,0,6); put(&m,7,4); put(&m,60,6); put(&m,0,1); put(&m,0,8); put(&m,0,1); put(&m,0,1); put(&m,0,1); put(&m,0,1);
  while (m.n % 8) put(&m,0,1);
  return m; }

/* ---- framing: returns NRZI-level symbols (+1/-1) ---- */
static int frame_symbols(const msg_t *m, int8_t *sym)
{
    uint8_t data[140]; int nb = (m->n + 7) / 8, nbits = 0, ones = 0, level = 1, ns = 0;
    uint8_t raw[2000];
    memcpy(data, m->b, (size_t)nb);
    uint16_t fcs = ais_crc16(data, nb);
    data[nb] = (uint8_t)fcs; data[nb + 1] = (uint8_t)(fcs >> 8);
    for (int i = 0; i < 24; i++) raw[nbits++] = (uint8_t)(i & 1);          /* training 0101... */
    for (int i = 0; i < 8; i++) raw[nbits++] = (uint8_t)((0x7E >> i) & 1);  /* start flag */
    for (int i = 0; i < (nb + 2) * 8; i++)
    {
        int bit = (data[i / 8] >> (i % 8)) & 1;                              /* LSB first */
        raw[nbits++] = (uint8_t)bit;
        if (bit) { if (++ones == 5) { raw[nbits++] = 0; ones = 0; } } else ones = 0;
    }
    for (int i = 0; i < 8; i++) raw[nbits++] = (uint8_t)((0x7E >> i) & 1);  /* end flag */
    for (int i = 0; i < 8; i++) raw[nbits++] = 0;                             /* ramp-down */
    for (int i = 0; i < nbits; i++) { if (raw[i] == 0) level = -level; sym[ns++] = (int8_t)level; } /* NRZI */
    return ns;
}

int main(int argc, char **argv)
{
    const char *out = argc > 1 ? argv[1] : "ais.cf32";
    int fs = argc > 2 ? atoi(argv[2]) : 192000;
    float snr = argc > 3 ? (float)atof(argv[3]) : 30.0f;
    float ferr = argc > 4 ? (float)atof(argv[4]) : 0.0f;
    int normal = argc > 5 && !strcmp(argv[5], "normal");
    const double sps = fs / 9600.0;
    /* AIS_SYNTH_REPS=n: repeat the 6-message schedule n times (1.2 s each,
     * jittered start times) for statistics */
    const int reps = getenv("AIS_SYNTH_REPS") ? atoi(getenv("AIS_SYNTH_REPS")) : 1;
    const double total_s = 1.2 * reps;
    const long n = (long)(total_s * fs);
    float *iq = calloc((size_t)n * 2, sizeof(float));
    /* schedule: channel A gets 1, 5, 4; channel B gets 18, 24A, 21 */
    msg_t (*ma[3])(void) = {type1, type5, type4}, (*mb[3])(void) = {type18, type24a, type21};
    double start_a[3] = {0.05, 0.40, 0.80}, start_b[3] = {0.10, 0.45, 0.70};
    /* Gaussian pulse, BT 0.4, span 3 symbols, integrated to a frequency pulse */
    const int gl = (int)(3 * sps) | 1; double *g = calloc((size_t)gl, sizeof(double)), gs = 0;
    for (int i = 0; i < gl; i++) { double t = (i - gl / 2) / sps, a = sqrt(2.0 / log(2.0)) * M_PI * 0.4; g[i] = exp(-(a * t) * (a * t)); gs += g[i]; }
    for (int i = 0; i < gl; i++) g[i] /= gs;

    srand(99);
    for (int r = 0; r < reps; r++)
    for (int chn = 0; chn < 2; chn++)
        for (int k = 0; k < 3; k++)
        {
            int8_t sym[2200]; msg_t m = (chn ? mb[k] : ma[k])(); int ns = frame_symbols(&m, sym);
            double jit = (reps > 1) ? (rand() % 1000) * 1e-5 : 0.0; /* 0..10 ms */
            long len = (long)(ns * sps) + gl, s0 = (long)((1.2 * r + (chn ? start_b[k] : start_a[k]) + jit) * fs);
            double *fr = calloc((size_t)len, sizeof(double)), ph = 0;
            for (long t = 0; t < (long)(ns * sps); t++) { int si = (int)(t / sps); for (int i = 0; i < gl; i++) fr[t + i] += sym[si] * g[i]; }
            /* A is 25 kHz below the LO; the board conjugates (A appears at +25 kHz) */
            double off = (chn == 0 ? -25000.0 : 25000.0) + ferr;
            if (!normal) off = -off;
            for (long t = 0; t < len && s0 + t < n; t++)
            {
                double dev = 2400.0 * fr[t] * (normal ? 1 : -1);
                ph += 2 * M_PI * (off + dev) / fs;
                iq[2 * (s0 + t)] += (float)cos(ph);
                iq[2 * (s0 + t) + 1] += (float)sin(ph);
            }
            free(fr);
        }
    {
        /* noise: SNR in a 25 kHz channel, per-channel carrier power 1 */
        float sigma = sqrtf(powf(10.0f, -snr / 10.0f) * ((float)fs / 25000.0f) / 2.0f);
        srand(7);
        for (long t = 0; t < 2 * n; t += 2)
        {
            float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f), u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f), r = sigma * sqrtf(-2 * logf(u1));
            iq[t] += r * cosf(2 * (float)M_PI * u2); iq[t + 1] += r * sinf(2 * (float)M_PI * u2);
        }
    }
    FILE *f = fopen(out, "wb"); fwrite(iq, sizeof(float), (size_t)n * 2, f); fclose(f);
    printf("%s: %.1f s at %d S/s, SNR %.1f dB, %+.0f Hz, %s orientation\n", out, total_s, fs, snr, ferr, normal ? "normal" : "board (conjugated)");
    return 0;
}
