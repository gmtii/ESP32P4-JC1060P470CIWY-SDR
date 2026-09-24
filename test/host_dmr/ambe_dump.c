/* Host-only: write every decoded AMBE frame in dsd-fme's .amb layout
 * (err byte, 6 bytes = bits 0..47, 1 byte = bit 48) to ambe_dump.bin. */
#include <stdio.h>
void dmr_voice_ambe_dump(const char ambe_d[49], int errs2)
{
    static FILE *f;
    unsigned char b;
    if (!f) f = fopen("ambe_dump.bin", "wb");
    fputc(errs2 & 0xFF, f);
    for (int i = 0; i < 6; i++) { b = 0; for (int j = 0; j < 8; j++) b = (unsigned char)((b << 1) | (ambe_d[8 * i + j] & 1)); fputc(b, f); }
    fputc(ambe_d[48] & 1, f);
    fflush(f);
}
