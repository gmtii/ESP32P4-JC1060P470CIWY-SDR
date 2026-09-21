
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stdbool.h"

#include "esp_dsp.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "math.h"

#include "sdr_math.h"
#include "ui.h"

#include "sdr.h"
#include "sdr_priv.h"
#include "rtl_source.h"
#include "audio_out.h"

#include "agc.h"
#include "nr.h"
#include "sam.h"
#include "nr_ss.h"

extern int nr_mode;
extern int agc_mode;
extern unsigned int time_sdrtask;
extern bool bucle;

extern bool f_nr;
extern bool f_nrss;
extern int f_autonotch_nr;

extern int filtro_indice;
extern bool f_actualiza;

// Generate Windowed-Sinc filter coefficients
void generate_FIR_coefficients(float *fir_coeffs, const unsigned int fir_len, const float ft)
{

    // Even or odd length of the FIR filter
    const bool is_odd = (fir_len % 2) ? (true) : (false);
    const float fir_order = (float)(fir_len - 1);

    // Window coefficients
    float *fir_window = (float *)malloc(fir_len * sizeof(float));
    dsps_wind_blackman_f32(fir_window, fir_len);

    for (int i = 0; i < fir_len; i++)
    {
        if ((i == fir_order / 2) && (is_odd))
        {
            fir_coeffs[i] = 2 * ft;
        }
        else
        {
            fir_coeffs[i] = sinf((2 * M_PI * ft * (i - fir_order / 2))) / (M_PI * (i - fir_order / 2));
        }

        fir_coeffs[i] *= fir_window[i];
    }

    free(fir_window);
}

float IRAM_ATTR alpha_beta_mag(float inphase, float quadrature)
// (c) András Retzler
// taken from libcsdr: https://github.com/simonyiszk/csdr
{
    // Min RMS Err      0.947543636291 0.392485425092
    // Min Peak Err     0.960433870103 0.397824734759
    // Min RMS w/ Avg=0 0.948059448969 0.392699081699
    const float alpha = 0.960433870103; // 1.0; //0.947543636291;
    const float beta = 0.397824734759;
    /* magnitude ~= alpha * max(|I|, |Q|) + beta * min(|I|, |Q|) */
    float abs_inphase = fabs(inphase);
    float abs_quadrature = fabs(quadrature);
    if (abs_inphase > abs_quadrature)
    {
        return alpha * abs_inphase + beta * abs_quadrature;
    }
    else
    {
        return alpha * abs_quadrature + beta * abs_inphase;
    }
}

void IRAM_ATTR sdrTask(void *args)
{

    esp_err_t ret = ESP_OK;

    sam_variables_init();
    dsps_fft2r_init_fc32(NULL, SAMPLE_BUFFER_SIZE);
    NR_SS_init();

    // Filtro biquad LPF 48000 x 0.15 para modos AM

    float coeffs_am[5];
    float w_lpf_i[5] = {0, 0};
    float w_lpf_q[5] = {0, 0};

    dsps_biquad_gen_lpf_f32(coeffs_am, 0.2, 1); // Q=3

    float w_hpf[5] = {0, 0};
    float w_lpf[5] = {0, 0};
    float w_notch[5] = {0, 0};

    float coeffs_hpf[5];
    float coeffs_lpf[5];

    // Filtros FIR de I para SSB
    dsps_fir_init_f32(&fir_i, CF_Re, history_i, 128);

    // Filtros FIR de Q para SSB
    dsps_fir_init_f32(&fir_q, CF_Im, history_q, 128);

    /* FIR MR para decimar/interpolar */

    float fird_coeffs[FIR_COEFFS_LEN];

    // dsps_firmr_init_f32(&firmr_i, (float *)fird_coeffs, firmr_i_State, FIR_COEFFS_LEN, 1, 4, 4);
    // dsps_firmr_init_f32(&firmr_q, (float *)fird_coeffs, firmr_q_State, FIR_COEFFS_LEN, 1, 4, 4);
    // dsps_firmr_init_f32(&firmr_p, (float *)fird_coeffs, firmr_p_State, FIR_COEFFS_LEN, 4, 1, 0);

    // FIRD

    /* Generador FIR */

    dsps_fird_init_f32(&fird_i, fird_coeffs, fird_delay_i, FIR_COEFFS_LEN, DR);
    dsps_fird_init_f32(&fird_q, fird_coeffs, fird_delay_q, FIR_COEFFS_LEN, DR);

    generate_FIR_coefficients(fird_coeffs, FIR_COEFFS_LEN, 0.75 / DR);

    int i = 0;

    while (1)
    {
        /* Get one block of 48 kSps I/Q from the RTL-SDR chain (USB Host).
         * The codec's I2S TX write below paces this loop; rtl_source absorbs
         * the dongle-vs-codec clock difference. */
        ret = rtl_source_read_float(i_sample, q_sample, SAMPLE_BUFFER_SIZE, 100);
        if (ret != ESP_OK)
        {
            /* No dongle or stalled stream: keep the codec fed with silence */
            memset(sampleData_out, 0, sizeof(sampleData_out));
            audio_out_write((const int16_t *)&sampleData_out[0].sample, SAMPLE_BUFFER_SIZE);
            continue;
        }

        unsigned int start_sdrtask = dsp_get_cpu_cycle_count();

        /* Vectores para FFT */
        memcpy(i_fft, i_sample, sizeof(i_fft));
        memcpy(q_fft, q_sample, sizeof(q_fft));

        if (demod_modo != DEMOD_FM)
        {
            // Ya estamos en CODEC_SAMPLERATE
            // Hago una conversion de frecuencia a SR/4
            // p.e. 192khz serán 48khz, por lo tanto 5.450 pasa a ser 5.402. Sintonizamos por abajo
            // pero presentamos la frecuencia con esa suma de SR/4.
            for (i = 0; i < SAMPLE_BUFFER_SIZE; i += 4)
            { // i_sample_d contains I = real values
                // i_sample_d contains Q = imaginary values
                // xnew(0) =  xreal(0) + jximag(0)
                // leave as it is!
                // xnew(1) =  - ximag(1) + jxreal(1)
                float hh1 = -q_sample[i + 1];
                float hh2 = i_sample[i + 1];
                i_sample[i + 1] = hh1;
                q_sample[i + 1] = hh2;
                // xnew(2) = -xreal(2) - jximag(2)
                hh1 = -i_sample[i + 2];
                hh2 = -q_sample[i + 2];
                i_sample[i + 2] = hh1;
                q_sample[i + 2] = hh2;
                // xnew(3) = + ximag(3) - jxreal(3)
                hh1 = q_sample[i + 3];
                hh2 = -i_sample[i + 3];
                i_sample[i + 3] = hh1;
                q_sample[i + 3] = hh2;
            }

            // dsps_firmr_f32(&firmr_i, i_sample, i_sample_out, SAMPLE_BUFFER_SIZE);
            // dsps_firmr_f32(&firmr_q, q_sample, q_sample_out, SAMPLE_BUFFER_SIZE);

            dsps_fird_f32_ansi(&fird_i, i_sample, i_sample_out_d, SAMPLE_BUFFER_SIZE / DR);
            dsps_fird_f32_ansi(&fird_q, q_sample, q_sample_out_d, SAMPLE_BUFFER_SIZE / DR);

            if (demod_modo == DEMOD_USB || demod_modo == DEMOD_LSB) // En AM/SAM/FM no aplicamos desfase a Q
            {
                dsps_fir_f32(&fir_i, i_sample_out_d, i_sample_out_d, SAMPLE_BUFFER_SIZE / DR);
                dsps_fir_f32(&fir_q, q_sample_out_d, q_sample_out_d, SAMPLE_BUFFER_SIZE / DR);

                if (demod_modo == DEMOD_LSB)
                {
                    dsps_add_f32(i_sample_out_d, q_sample_out_d, demod_out_d, SAMPLE_BUFFER_SIZE / DR, 1, 1, 1); // Demodula USB
                }
                else if (demod_modo == DEMOD_USB)
                {
                    dsps_sub_f32(i_sample_out_d, q_sample_out_d, demod_out_d, SAMPLE_BUFFER_SIZE / DR, 1, 1, 1); // Demodula LSB
                }
            }
            else if (demod_modo >= DEMOD_SAM && demod_modo <= DEMOD_SAMU)
                SAM(i_sample_out_d, q_sample_out_d, demod_out_d, SAMPLE_BUFFER_SIZE / DR, demod_modo);
            else if (demod_modo == DEMOD_AM)
            {
                for (i = 0; i < SAMPLE_BUFFER_SIZE / DR; i++)
                {
                    audiotmp = alpha_beta_mag(i_sample_out_d[i], q_sample_out_d[i]);
                    w = audiotmp + wold * 0.9999f; // yes, I want a superb bass response ;-)
                    demod_out_d[i] = w - wold;
                    wold = w;
                }
            }
        }
        else
        {
            float angle, x, y;

            for (i = 0; i < SAMPLE_BUFFER_SIZE; i++)
            {
                y = (q_sample[i] * fm_variables.i_sample_prev) - (i_sample[i] * fm_variables.q_sample_prev);
                x = (i_sample[i] * fm_variables.i_sample_prev) + (q_sample[i] * fm_variables.q_sample_prev);

                angle = ApproxAtan2(y, x);

                if (isnanf(angle))
                {
                    angle = 0.0f;
                }

                demod_out[i] = (float)(angle / M_PI) * 0.1f;

                fm_variables.q_sample_prev = q_sample[i]; // save "previous" value of each channel to allow detection of the change of angle in next go-around
                fm_variables.i_sample_prev = i_sample[i];
            }
        }

        /* Pongo entrada en salida */

        if (1)
        {

            if (demod_modo != DEMOD_FM)
            {

                if (f_actualiza)
                {
                    // int f_alta, f_baja;

                    f_actualiza = false;

                    switch (filtro_indice)
                    {
                    case F_CW:
                        currentVFO.f_alta = 500;
                        currentVFO.f_baja = 100;
                        break;

                    case F_1K8:
                        currentVFO.f_alta = 1800;
                        currentVFO.f_baja = 100;
                        break;

                    case F_2K3:
                        currentVFO.f_alta = 2300;
                        currentVFO.f_baja = 100;
                        break;

                    case F_3K6:
                        currentVFO.f_alta = 2700;
                        currentVFO.f_baja = 100;
                        break;

                    case F_VAR:
                        currentVFO.f_alta = 3600;
                        currentVFO.f_baja = 100;
                        break;

                    default:
                        currentVFO.f_alta = 2700;
                        currentVFO.f_baja = 100;
                        break;
                    }

                    // Filtro variable

                    dsps_biquad_gen_hpf_f32(coeffs_hpf, (float)currentVFO.f_baja / (SAMPLE_RATE / DR), 1);
                    dsps_biquad_gen_lpf_f32(coeffs_lpf, (float)currentVFO.f_alta / (SAMPLE_RATE / DR), 1);
                }

                dsps_biquad_f32(demod_out_d, demod_out_d, SAMPLE_BUFFER_SIZE / DR, coeffs_lpf, w_lpf);
                dsps_biquad_f32(demod_out_d, demod_out_d, SAMPLE_BUFFER_SIZE / DR, coeffs_hpf, w_hpf);

                if (f_nr && !f_nrss)
                    NR(demod_out_d, SAMPLE_BUFFER_SIZE / DR, f_autonotch_nr);

                if (f_nrss && !f_nr)
                    NR_SS(demod_out_d, SAMPLE_BUFFER_SIZE / DR);

                RxAGC(demod_out_d, SAMPLE_BUFFER_SIZE / DR);

                // dsps_firmr_f32(&firmr_p, demod_out_d, demod_out, SAMPLE_BUFFER_SIZE / DR);

                // Up sampLing (Interpolation)
                for (i = SAMPLE_BUFFER_SIZE / DR - 1; i >= 0; i--)
                {
                    demod_out[i * DR] = DR * demod_out_d[i];
                    for (int j = 1; j < DR; j++)
                    {
                        demod_out[i * DR + j] = 0;
                    }
                }

                // Anti-aliasing Filter
                dsps_biquad_f32(demod_out, demod_out, SAMPLE_BUFFER_SIZE, RX_biquad0, z_IIR0);
                dsps_biquad_f32(demod_out, demod_out, SAMPLE_BUFFER_SIZE, RX_biquad1, z_IIR1);
                dsps_biquad_f32(demod_out, demod_out, SAMPLE_BUFFER_SIZE, RX_biquad2, z_IIR2);
                dsps_biquad_f32(demod_out, demod_out, SAMPLE_BUFFER_SIZE, RX_biquad3, z_IIR3);
            }

            for (i = 0; i < SAMPLE_BUFFER_SIZE; i++) // convierte a int16
            {
                sampleData_out[i].ch[0] = (int16_t)(demod_out[i] * (float)INT16_MAX);
                sampleData_out[i].ch[1] = sampleData_out[i].ch[0]; // segundo canal para el SFM
            }
        }
        else
        {
            for (i = 0; i < SAMPLE_BUFFER_SIZE; i++)
            {
                sampleData_out[i].ch[0] = sampleData_in[i].ch[0];
                sampleData_out[i].ch[1] = sampleData_in[i].ch[1];
            }
        }

        time_sdrtask = dsp_get_cpu_cycle_count() - start_sdrtask;

        if (debug)
        {
            // Serial.printf("SDR Task cycles: %u\n", dsp_get_cpu_cycle_count() - start_sdrtask);
        }

        // Envia el DAC SAMPLE_BUFFER_SIZE * 4 ( 2 canales, 16 bit cada uno)
        ret = audio_out_write((const int16_t *)&sampleData_out[0].sample, SAMPLE_BUFFER_SIZE);
    }

    // vTaskDelete(NULL);
}

void shift_right_circular(int16_t *v, size_t size, int offset)
{
    if (size == 0 || offset == 0)
        return;
    offset %= size;
    int16_t tmp[offset];
    memcpy(tmp, &v[size - offset], offset * sizeof(int16_t));
    memmove(&v[offset], v, (size - offset) * sizeof(int16_t));
    memcpy(v, tmp, offset * sizeof(int16_t));
}

void IRAM_ATTR calcula_fft(void)
{
    int N = SAMPLE_BUFFER_SIZE;

    // save old pixels for lowpass filter
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        pixelold[i] = pixelnew[i];
    }

    // Generate hann window
    dsps_wind_hann_f32(wind, N);

    // Convert two input vectors to one complex vector i,q
    for (int i = 0; i < N; i++)
    {
        fft_vector[i * 2 + 0] = i_fft[i] * wind[i];
        fft_vector[i * 2 + 1] = q_fft[i] * wind[i];
    }

    // FFT
    dsps_fft2r_fc32_arp4(fft_vector, N);
    //  Bit reverse
    dsps_bit_rev_fc32(fft_vector, N);

    // calculate mag = I*I + Q*Q,
    // and simultaneously put them into the right order
    for (int i = 0; i < N / 2; i++)
    {
        fft_mag[i + N / 2] = (fft_vector[i * 2] * fft_vector[i * 2] + fft_vector[i * 2 + 1] * fft_vector[i * 2 + 1]);
        fft_mag[i + 0] = (fft_vector[(i + N / 2) * 2] * fft_vector[(i + N / 2) * 2] + fft_vector[(i + N / 2) * 2 + 1] * fft_vector[(i + N / 2) * 2 + 1]);
    }

    int spec_min = 0;
    int spec_max = 0;

    for (int i = 0; i < N; i++)
    {
        fft_mag[i] = 0.6 * fft_mag[i] + 0.4 * fft_mag_old[i];
        fft_mag_old[i] = fft_mag[i];
        pixelnew[N - 1 - i] = 20 * log10f_fast(fft_mag[i] * (float)(32768.0f));

        if (spec_min > pixelnew[i]) // Calcula el valor mínimo del vector pixelnew
            spec_min = pixelnew[i];

        if (spec_max < pixelnew[i]) // valor máximo
            spec_max = pixelnew[i];
    }

    spec_offset = (spec_offset + 9 * spec_offset_old) / 10;

    if (spec_min < -15 && spec_offset < 5) // estamos muy abajo, subimos el espectro a ritmo de *spec_agc
        spec_offset += 3 * spec_agc;
    else if (spec_min < 0 && spec_offset < 25 && spec_rebote++ > 3) // no tan abajo, subimos a ritmo spec_agc
    {
        spec_offset += spec_agc;
        spec_rebote = 0;
    }
    else if (spec_max > WAVEFORM_HEIGHT / 2 && spec_rebote++ > 3) // muy altos, bajamos a ritmo de spec_agc
    {
        spec_offset -= spec_agc;
        spec_rebote = 0;
    }

    if (spec_offset > WAVEFORM_HEIGHT / 2)
        spec_offset = WAVEFORM_HEIGHT / 2; // corrección para el caso de offset disparado

    spec_offset_old = spec_offset;

    for (int i = 0; i < N; i++) // aplica el "offset" al espectro para ajustar
        pixelnew[i] += spec_offset;

    // Rota 128 a la derecha para corregir el problema con el CANVAS dichoso de LGVL

    // shift_right_circular(pixelnew, N, 128);
}
