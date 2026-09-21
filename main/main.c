/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "esp_memory_utils.h"
#include "esp_dsp.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "bsp_board_extra.h"

#include "iot_knob.h"
#include "esp_ldo_regulator.h"
#include "i2s_driver.h"
#include "nau8822.h"
#include "ui.h"
#include "sdr.h"
#include "agc.h"
#include "msi001.h"
#include "uart_commands.h"

#include "pins_config.h"

static char *TAG = "MAIN";
esp_ldo_channel_handle_t ldo3 = NULL;

const char *demod_modos_texto[7] = {
    "USB ", // 0
    "LSB ",
    "AM  ",
    "SAM ",
    "S-L ",
    "S-U ",
    "FM  " // 6
};

const char *pasos_texto[7] = {
    "0.1 k", // 0
    " 1 k ",
    " 5 k ",
    " 10 k",
    "0.1 M",
    " 1 M ",
};

int pasos_indice = 1;
int pasos[6] = {
    100,
    1000,
    5000,
    10000,
    100000,
    1000000};

bool f_actualiza = true;
int filtro_indice = 2;
const char *filtros_texto[5] = {
    "CW",
    "1800",
    "2300",
    "2700",
    "3600"};

const char *agc_texto[6] = {

    "AGC OFF",
    "AGC CST",
    "AGC LNG",
    "AGC SLW",
    "AGC MED",
    "AGC FST"};

VFO currentVFO = {.VFOName = "VFO-A", .Frec = 7350000, .demod_modo = DEMOD_LSB, .filtro = 0, .step = 1000, .AGC = true, .FLT = false, .SPLT = false, .NR_SS = false, .NR = false, .ANR = 0, .f_baja = 300, .f_alta = 2700};

bool debug = false;
bool nr_ss_debug;
bool bucle = false;
bool screen_update = true;
unsigned int time_sdrtask = 0;
int demod_modo = DEMOD_LSB;
int volume = 0x1F;

bool f_nr = false;
bool f_nrss = false;
int f_autonotch_nr = 1;
int V_Rsv = 0;

bool rebote_knob = false;
bool vfo_update = false;

static void setup_ldo(void)
{
    // Create configuration for LDO index 3
    esp_ldo_channel_config_t config3 = {
        .chan_id = 4, // discovered by trial and error
        .voltage_mv = 3300,
        .flags = {
            .adjustable = 1,
            .owned_by_hw = 0,
            .bypass = 0}};

    if (esp_ldo_acquire_channel(&config3, &ldo3) == ESP_OK)
    {
        ESP_LOGI(TAG, "LDO index 3 acquired");
    }
    else
    {
        ESP_LOGI(TAG, "Failed to acquire LDO index 3");
    }

    if (ldo3)
    {
        esp_ldo_channel_adjust_voltage(ldo3, 3300);
        ESP_LOGI(TAG, "LDO index 3 voltage adjusted");
    }

    // Optionally: dump to see results
    esp_ldo_dump(stdout);
}

static knob_handle_t s_knob;
int32_t s_enc_diff = 0; // acumulador para LVGL

#define PIN_COUNT 3
int pins[PIN_COUNT] = {20, 32, 33};

void test_gpio(void)
{
    // Configurar GPIO como salida
    for (int i = 0; i < PIN_COUNT; i++)
    {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << pins[i],
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&cfg);
    }

    const int half_period_us = 500; // 500 us → 1 kHz señal completa (1 ms)

    while (1)
    {
        // HIGH
        for (int i = 0; i < PIN_COUNT; i++)
        {
            gpio_set_level(pins[i], 1);
        }
        vTaskDelay(half_period_us);

        // LOW
        for (int i = 0; i < PIN_COUNT; i++)
        {
            gpio_set_level(pins[i], 0);
        }
        vTaskDelay(half_period_us);
    }
}

static void _knob_left_cb(void *arg, void *data)
{
    if (!rebote_knob)
    {
        s_enc_diff--; // LVGL interpreta <0 como giro antihorario
    }
    else
        rebote_knob = false;
}

static void _knob_right_cb(void *arg, void *data)
{
    if (!rebote_knob)
    {
        s_enc_diff++; // LVGL interpreta >0 como giro horario
    }
    else
        rebote_knob = false;
}

void create_knob()
{

    // create knob
    knob_config_t cfg = {
        .default_direction = 0,
        .gpio_encoder_a = GPIO_KNOB_A,
        .gpio_encoder_b = GPIO_KNOB_B,
    };
    s_knob = iot_knob_create(&cfg);
    if (NULL == s_knob)
    {
        ESP_LOGE(TAG, "knob create failed");
    }
    else
    {

        iot_knob_register_cb(s_knob, KNOB_LEFT, _knob_left_cb, NULL);
        iot_knob_register_cb(s_knob, KNOB_RIGHT, _knob_right_cb, NULL);
    }
}

void app_main(void)
{
    // ESP_ERROR_CHECK(bsp_extra_codec_init());

    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = 1024 * 50,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
            .sw_rotate = false,
        }};
    bsp_display_start_with_config(&cfg);
    bsp_display_backlight_on();
    bsp_display_brightness_set(5);

    setup_ldo();

    create_knob();

    i2s_driver_init(SAMPLE_RATE);

    nau8822_init(0); // Modo de inicialización del NAU8822

    nau8822_init(7); // LIN RIN
    // nau8822_init(8);

    nau8822_spk_volume(0x20);

    init_msi001();

    /* AGC */
    AGC_init();
    AGC_prep();

    bsp_i2c_init();

    bsp_display_lock(0);

    init_ui();

    bsp_display_unlock();

    xTaskCreatePinnedToCore(sdrTask, "sdrTask", 4096, NULL, 20, NULL, 1);

    uart_init();
    xTaskCreate(uart_command_loop, "uart_cmd_loop", 4096, NULL, 5, NULL);

    lvgl_encoder_init(); // input device LVGL

    xTaskCreate(tarea_encoder, "tarea_encoder", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Main finalizado.");
}
