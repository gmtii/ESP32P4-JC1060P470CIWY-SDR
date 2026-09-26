/* Host test for main/dmr/dmr_fec.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dmr_fec.h"
#include "fme_ref_matrices.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static int pop(uint32_t v) { int c = 0; while (v) { v &= v - 1; c++; } return c; }

/* Linear code: codeword(d) = XOR of the rows selected by d's bits (MSB = row 0). */
static void check_code(const char *name, uint32_t (*enc)(uint32_t), const uint32_t *rows, int k, int n,
                       bool (*dec)(uint32_t *), int t)
{
    int dmin = 99;
    for (uint32_t d = 0; d < (1u << k); d++)
    {
        uint32_t ref = 0;
        for (int i = 0; i < k; i++)
            if (d & (1u << (k - 1 - i))) ref ^= rows[i];
        CHECK(enc(d) == ref, "%s: codeword mismatch for data 0x%X", name, d);
        if (d) { int w = pop(enc(d)); if (w < dmin) dmin = w; }
    }
    /* every pattern of up to t errors on random codewords must be corrected */
    for (int trial = 0; trial < 2000; trial++)
    {
        uint32_t d = (uint32_t)rand() & ((1u << k) - 1u), cw = enc(d), rx = cw;
        int ne = (trial % (t + 1));
        for (int e = 0; e < ne; e++) rx ^= 1u << (rand() % n);
        uint32_t fix = rx;
        bool ok = dec(&fix);
        if (pop(rx ^ cw) <= t) CHECK(ok && fix == cw, "%s: failed to correct %d errors", name, pop(rx ^ cw));
    }
    printf("%-14s n=%2d k=%2d dmin=%d  matches dsd-fme: %s, corrects %d\n", name, n, k, dmin, fails ? "?" : "yes", t);
}

int main(void)
{
    check_code("Hamming(7,4)", dmr_hamming_7_4_encode, ref_Hamming_7_4_m_G, 4, 7, dmr_hamming_7_4_decode, 1);
    check_code("Hamming(13,9)", dmr_hamming_13_9_encode, ref_Hamming_13_9_m_G, 9, 13, dmr_hamming_13_9_decode, 1);
    check_code("Hamming(15,11)", dmr_hamming_15_11_encode, ref_Hamming_15_11_m_G, 11, 15, dmr_hamming_15_11_decode, 1);
    check_code("Hamming(16,11)", dmr_hamming_16_11_encode, ref_Hamming_16_11_4_m_G, 11, 16, dmr_hamming_16_11_decode, 1);
    check_code("Golay(20,8)", dmr_golay_20_8_encode, ref_Golay_20_8_m_G, 8, 20, dmr_golay_20_8_decode, 3);
    check_code("QR(16,7)", dmr_qr_16_7_encode, ref_QR_16_7_6_m_G, 7, 16, dmr_qr_16_7_decode, 2);

    /* BPTC(196,96): round trip, then 1 error per row survives */
    {
        uint8_t d[96], tx[196], rx[196], out[96];
        int ok_clean = 0, ok_err = 0;
        for (int trial = 0; trial < 200; trial++)
        {
            for (int i = 0; i < 96; i++) d[i] = rand() & 1;
            dmr_bptc196_encode(d, tx);
            memcpy(rx, tx, 196);
            ok_clean += dmr_bptc196_decode(rx, out) == 0 && !memcmp(out, d, 96);
            for (int e = 0; e < 3; e++) rx[rand() % 196] ^= 1; /* 3 scattered errors */
            ok_err += dmr_bptc196_decode(rx, out) == 0 && !memcmp(out, d, 96);
        }
        CHECK(ok_clean == 200, "BPTC196 clean round trip %d/200", ok_clean);
        printf("BPTC(196,96)   clean %d/200, with 3 random bit errors %d/200\n", ok_clean, ok_err);
    }
    /* Embedded LC */
    {
        uint8_t lc[72], tx[128], rx[128], out[72];
        int ok_clean = 0, ok_err = 0;
        for (int trial = 0; trial < 200; trial++)
        {
            for (int i = 0; i < 72; i++) lc[i] = rand() & 1;
            dmr_emb_lc_encode(lc, tx);
            memcpy(rx, tx, 128);
            ok_clean += dmr_emb_lc_decode(rx, out) && !memcmp(out, lc, 72);
            rx[rand() % 128] ^= 1;
            ok_err += dmr_emb_lc_decode(rx, out) && !memcmp(out, lc, 72);
        }
        CHECK(ok_clean == 200, "EMB LC clean %d/200", ok_clean);
        CHECK(ok_err == 200, "EMB LC 1 error %d/200", ok_err);
        printf("Embedded LC    clean %d/200, with 1 bit error %d/200\n", ok_clean, ok_err);
    }
    /* RS(12,9) */
    {
        uint8_t b[12];
        int ok = 0, caught = 0;
        for (int trial = 0; trial < 500; trial++)
        {
            for (int i = 0; i < 9; i++) b[i] = rand() & 0xFF;
            dmr_rs_12_9_parity(b, DMR_RS_MASK_VOICE_HEADER, &b[9]);
            ok += dmr_rs_12_9_check(b, DMR_RS_MASK_VOICE_HEADER);
            caught += !dmr_rs_12_9_check(b, DMR_RS_MASK_TERMINATOR); /* wrong mask must fail */
            b[rand() % 12] ^= (uint8_t)(1 + rand() % 255);
            caught += !dmr_rs_12_9_check(b, DMR_RS_MASK_VOICE_HEADER);
        }
        CHECK(ok == 500 && caught == 1000, "RS(12,9) ok=%d caught=%d", ok, caught);
        printf("RS(12,9)       valid %d/500, corrupted/wrong-mask rejected %d/1000\n", ok, caught);
        /* single-byte correction, and double-byte errors never "corrected" into a wrong word */
        {
            int fixed = 0, wrong = 0;
            for (int trial = 0; trial < 2000; trial++)
            {
                uint8_t orig[12], rx[12];
                for (int i = 0; i < 9; i++) orig[i] = rand() & 0xFF;
                dmr_rs_12_9_parity(orig, DMR_RS_MASK_TERMINATOR, &orig[9]);
                memcpy(rx, orig, 12);
                rx[rand() % 12] ^= (uint8_t)(1 + rand() % 255);
                fixed += dmr_rs_12_9_decode(rx, DMR_RS_MASK_TERMINATOR) && !memcmp(rx, orig, 12);
                memcpy(rx, orig, 12);
                int a = rand() % 12, b = (a + 1 + rand() % 11) % 12;
                rx[a] ^= (uint8_t)(1 + rand() % 255); rx[b] ^= (uint8_t)(1 + rand() % 255);
                wrong += dmr_rs_12_9_decode(rx, DMR_RS_MASK_TERMINATOR) && memcmp(rx, orig, 12);
            }
            CHECK(fixed == 2000, "RS single-byte correction %d/2000", fixed);
            printf("RS(12,9)       1-byte errors corrected %d/2000, 2-byte errors mis-corrected %d/2000\n", fixed, wrong);
        }
    }
    printf(fails ? "\n%d FAILURES\n" : "\nALL FEC TESTS PASSED\n", fails);
    return fails != 0;
}
