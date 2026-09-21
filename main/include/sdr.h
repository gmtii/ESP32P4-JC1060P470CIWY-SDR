
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define SAMPLE_BUFFER_SIZE (1024)

#define DR 4 // decimation factor
#define SAMPLE_RATE (48000)
#define FREQ_CONV_OFFSET (SAMPLE_RATE / DR)

#define WAVEFORM_WIDTH SAMPLE_BUFFER_SIZE
#define WAVEFORM_HEIGHT 192
#define WATERFALL_HEIGHT 128

#define DEMOD_USB 0
#define DEMOD_LSB 1
#define DEMOD_AM 2
#define DEMOD_SAM 3
#define DEMOD_SAML 4
#define DEMOD_SAMU 5
#define DEMOD_FM 6

#define F_CW 0
#define F_1K8 1
#define F_2K3 2
#define F_3K6 3
#define F_VAR 4

typedef struct
{
    char *VFOName;  // Nombre del VFO
    uint32_t Frec;  // Frecuencia en Hz
    int demod_modo; // Modo de demodulación
    int filtro;
    int step;
    bool AGC;
    bool FLT;
    bool SPLT;
    bool NR_SS;
    bool NR;
    int ANR;
    int f_baja; // Valor filtro Baja
    int f_alta; // Valor filtro Alta
} VFO;

extern VFO currentVFO;
extern int filtro;
extern bool f_actualiza;

void sdrTask(void *args);
void calcula_fft(void);

#ifdef __cplusplus
}
#endif