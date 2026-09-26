/* Host test: ./ais_host_test file.cf32   (192 kS/s interleaved float I/Q, as the
 * firmware's wide path delivers it). Prints decoded messages and statistics. */
#include <stdio.h>
#include <stdlib.h>
#include "ais_demod.h"
#include "ais_msg.h"
static void pr(const char *l) { printf("%s\n", l); }
static void on_frame(const ais_frame_t *f, void *ctx) { (void)ctx; ais_msg_handle(f); }
int main(int argc, char **argv)
{
    FILE *f; long sz, n; float *buf, *bi, *bq;
    if (argc < 2) { fprintf(stderr, "usage: %s file.cf32\n", argv[0]); return 1; }
    f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END); sz = ftell(f); fseek(f, 0, SEEK_SET); n = sz / 8;
    buf = malloc(sz); bi = malloc(n * 4); bq = malloc(n * 4);
    if (fread(buf, 1, sz, f) != (size_t)sz) return 1;
    for (long k = 0; k < n; k++) { bi[k] = buf[2 * k]; bq[k] = buf[2 * k + 1]; }
    ais_msg_reset(); ais_msg_set_printer(pr); ais_demod_init(on_frame, NULL);
    for (long k = 0; k < n; k += 4096) ais_demod_feed(bi + k, bq + k, (int)(n - k < 4096 ? n - k : 4096));
    { ais_demod_stats_t d; ais_msg_stats_t m; ais_demod_get_stats(&d); ais_msg_get_stats(&m);
      printf("frames ok A %u B %u | CRC errors A %u B %u | decoded %u unknown %u\n", d.frames_ok[0], d.frames_ok[1], d.crc_errors[0], d.crc_errors[1], m.decoded, m.unknown); }
    return 0;
}
