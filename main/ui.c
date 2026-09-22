#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "sdr.h"
#include "ui.h"
#include "ui_priv.h"
#include "agc.h"
#include "audio_out.h"
#include "smeter.h"
#include "math.h"

#include "rtl_source.h"
#include "esp_timer.h"

/* Uncomment to log how long the FFT, spectrum and waterfall steps take (every 2 s) */
// #define UI_PERF_LOG

#include "images/smeter2.c"

#include "menu.h"

extern VFO currentVFO;

static char *TAG = "UI";

#define SMETER_MIN_ANGLE 135 // grados
#define SMETER_MAX_ANGLE 405 // grados
#define SMETER_RANGE 270     // arco total

static float smeter_value = 0.0f; // valor S (0..9)

extern unsigned int time_sdrtask;
int loops = 0;

extern int demod_modo;
extern int f_nrss;
extern agc_wdsp_params_t agc_wdsp_conf;
extern bool screen_update;

extern int pasos[6];
extern int pasos_indice;

extern bool f_actualiza;
extern int filtro_indice;
extern char *filtros_texto[5];
extern char *agc_texto[6];

extern char *demod_modos_texto[7];
extern char *pasos_texto[6];

static bool bloqueo_pulsacion = false;

static bool refresca_smeter = false;
int contador_printf = 0;

extern int32_t s_enc_diff;

static const char *TAG_MEM = "MEM";

/* Imprime estado de memoria LVGL + ESP32 */
void debug_print_mem(const char *tag)
{
  lv_mem_monitor_t m;
  lv_mem_monitor(&m);

  size_t free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t largest_8bit = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

  ESP_LOGI(TAG_MEM, "---- MEM DEBUG: %s ----", tag);

  /* LVGL */
  ESP_LOGI(TAG_MEM,
           "LVGL: total=%lu free=%lu frag=%lu%%",
           (unsigned long)m.total_size,
           (unsigned long)m.free_size,
           (unsigned long)m.frag_pct);

  /* ESP32 heap */
  ESP_LOGI(TAG_MEM,
           "HEAP 8BIT: free=%lu largest_block=%lu",
           (unsigned long)free_8bit,
           (unsigned long)largest_8bit);

  ESP_LOGI(TAG_MEM, "-----------------------------");
}

static void desbloquear_cb(lv_timer_t *timer)
{
  bloqueo_pulsacion = false;
  lv_timer_pause(timer_debounce); // 🔥 Esto elimina el timer después de ejecutarse

  ESP_LOGI(TAG, "Debounce...");
}

void refresca_VFO(void)
{
  // Proteger LVGL si usas esp_lvgl_port
  if (lvgl_port_lock(0))
  {
    lv_label_set_text_fmt(freq_label, "%d%d.%d%d%d.%d%d%d",
                          (currentVFO.Frec % 100000000) / 10000000,
                          (currentVFO.Frec % 10000000) / 1000000,
                          (currentVFO.Frec % 1000000) / 100000,
                          (currentVFO.Frec % 100000) / 10000,
                          (currentVFO.Frec % 10000) / 1000,
                          (currentVFO.Frec % 1000) / 100,
                          (currentVFO.Frec % 100) / 10,
                          (currentVFO.Frec % 10) / 1);
    lvgl_port_unlock();
  }
}

void timer_dibuja_pantalla(lv_timer_t *timer)
{
  if (screen_update)
  {
#ifdef UI_PERF_LOG
    static int64_t acc_fft, acc_spec, acc_wf;
    static uint32_t n_frames;
    const int64_t t0 = esp_timer_get_time();
#endif
    calcula_fft();
#ifdef UI_PERF_LOG
    const int64_t t1 = esp_timer_get_time();
#endif
    refresca_smeter = true;

    /* Task lock */
    if (lvgl_port_lock(0))
    {
      spectrum();
#ifdef UI_PERF_LOG
      const int64_t t2 = esp_timer_get_time();
#endif
      waterfall_update();
#ifdef UI_PERF_LOG
      const int64_t t3 = esp_timer_get_time();
      acc_fft += t1 - t0;
      acc_spec += t2 - t1;
      acc_wf += t3 - t2;
      if (++n_frames >= 60)
      {
        ESP_LOGI("PERF", "per frame: fft %d us, spectrum %d us, waterfall %d us",
                 (int)(acc_fft / n_frames), (int)(acc_spec / n_frames), (int)(acc_wf / n_frames));
        acc_fft = acc_spec = acc_wf = 0;
        n_frames = 0;
      }
#endif
      lvgl_port_unlock();
    }
  }
}

void timer_smeter_update(lv_timer_t *timer)
{
  if (refresca_smeter && lvgl_port_lock(0))
  {
    smeter_set_dbm(calculadBm());
    refresca_smeter = false;
    lvgl_port_unlock();
  }
}

void timer_uso_cpu(lv_timer_t *timer)
{

  return;
  /* Task lock */
  if (lvgl_port_lock(0))
  {

    lv_label_set_text_fmt(label5, "%u", time_sdrtask);
    lv_label_set_text_fmt(label6, "%u", esp_get_free_heap_size() / 1000);

    lvgl_port_unlock();
  }

  // loops++;
  // ESP_LOGI(TAG, "loops= %d --- sdrtask= %u", loops, time_sdrtask);
}

static inline void lv_draw_pixel(int x, int y, uint16_t color)
{
  waveformbuffer[y * CANVAS_W + x] = color;
}

static inline void lv_draw_vline(int x, int y, int h, uint16_t color)
{
  // if ((unsigned)x >= CANVAS_W)
  //   return;
  // if (h <= 0)
  //   return;

  // int max_h = CANVAS_H - y;
  // if (max_h <= 0)
  //   return;
  // if (h > max_h)
  //   h = max_h;

  uint32_t index = y * CANVAS_W + x;
  for (int i = 0; i < h; i++)
  {
    waveformbuffer[index] = color;
    index += CANVAS_W;
  }
}

/*
 * 5-point smoothing with weights 0.50 / 0.18 / 0.07, in integer math (64/23/9 out of 128).
 * The P4 FPU is single precision only: the previous expression used double constants,
 * so every column ran ~18 soft-float double operations, twice per column, every frame.
 */
static inline int smooth5(const int16_t *p, int x)
{
  return (64 * p[x] + 23 * (p[x - 1] + p[x + 1]) + 9 * (p[x - 2] + p[x + 2])) >> 7;
}

void spectrum(void)
{

  lv_obj_invalidate(waveform_canvas);

  int16_t y_old, y_new, y1_new, y1_old;
  int16_t y1_old_minus = 0;
  int16_t y1_new_minus = 0;

#define MARGEN_DERECHO 2
#define MARGEN_IZQUIERDO 2

  for (int16_t x = MARGEN_IZQUIERDO; x < SAMPLE_BUFFER_SIZE - MARGEN_DERECHO; x++)
  {

    // moving window - weighted average of 5 points of the spectrum to smooth spectrum in the frequency domain
    // weights:  x: 50% , x-1/x+1: 36%, x+2/x-2: 14%

    y_new = smooth5(pixelnew, x);
    y_old = smooth5(pixelold, x);

    if (y_old > (spectrum_height - 1))
    {
      y_old = (spectrum_height - 1);
    }

    if (y_new > (spectrum_height - 1))
    {
      y_new = (spectrum_height - 1);
    }

    if (y_old < 0)
      y_old = 0;
    if (y_new < 0)
      y_new = 0;

    y1_old = (spectrum_y + spectrum_height - 1) - y_old;
    y1_new = (spectrum_y + spectrum_height - 1) - y_new;

    if (x == MARGEN_IZQUIERDO)
    {
      y1_old_minus = y1_old;
      y1_new_minus = y1_new;
    }
    if (x == SAMPLE_BUFFER_SIZE - MARGEN_DERECHO)
    {
      y1_old_minus = y1_old;
      y1_new_minus = y1_new;
    }

    // DELETE OLD LINE/POINT
    if (y1_old - y1_old_minus > 1)
    { // plot line upwards
      lv_draw_vline(x + spectrum_x, y1_old_minus + 1, y1_old - y1_old_minus, 0);
    }
    else if (y1_old - y1_old_minus < -1)
    { // plot line downwards
      lv_draw_vline(x + spectrum_x, y1_old, y1_old_minus - y1_old, 0);
    }
    else
    {
      lv_draw_pixel(x + spectrum_x, y1_old, 0); // delete old pixel
    }

    // DRAW NEW LINE/POINT
    if (y1_new - y1_new_minus > 1)
    { // plot line upwards
      lv_draw_vline(x + spectrum_x, y1_new_minus + 1, y1_new - y1_new_minus, 0x07e0);
    }
    else if (y1_new - y1_new_minus < -1)
    { // plot line downwards
      lv_draw_vline(x + spectrum_x, y1_new, y1_new_minus - y1_new, 0x07e0);
    }
    else
    {
      lv_draw_pixel(x + spectrum_x, y1_new, 0x07e0); // write new pixel
    }

    y1_new_minus = y1_new;
    y1_old_minus = y1_old;
  }

  for (int16_t x = 0; x < SAMPLE_BUFFER_SIZE; x += 128)
  {
    lv_draw_vline(x, 0, H, 0x31A6);
  }

  // Dibuja centro de espectro de demodulación (F-F/4 OFFSET)
  lv_draw_vline(W - W / 4, 0, H, 0x075f);
}

static void slider_changed_cb(lv_event_t *e)
{
  lv_obj_t *s = lv_event_get_target_obj(e);
  int32_t v = lv_slider_get_value(s);

  audio_out_set_volume((int)v);

  lv_label_set_text_fmt(value_label, "%d", v);
}

void btn_event_cb(lv_event_t *e)
{
  if (bloqueo_pulsacion)
    return; // Ignora si está en cooldown

  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target_obj(e);

  if (code == LV_EVENT_CLICKED)
  {
    bloqueo_pulsacion = true;

    if (obj == btn_modos)
    {
      demod_modo--;
      if (demod_modo < 0)
        demod_modo = 5;
      lv_label_set_text_fmt(label_modos, "%s", demod_modos_texto[demod_modo]);
      dibuja_pasabanda();
      refresca_indicadores();
    }
    else if (obj == btn_menu || obj == waterfall_canvas)
    {
      if (timer_pantalla)
        lv_timer_pause(timer_pantalla);
      if (timer_smeter)
        lv_timer_pause(timer_smeter);

      debug_print_mem("ANTES de crear menu");
      ui_create_control_panel();
      debug_print_mem("DESPUES de crear menu");
    }
    else if (obj == btn_step)
    {
      pasos_indice++;
      if (pasos_indice > 5)
        pasos_indice = 0;

      lv_label_set_text_fmt(label_step, "%s", pasos_texto[pasos_indice]);
      refresca_indicadores();
    }
    else if (obj == btn_filtros)
    {

      filtro_indice++;
      if (filtro_indice > 4)
        filtro_indice = 0;
      f_actualiza = true;

      dibuja_pasabanda();
      refresca_indicadores();

      lv_label_set_text_fmt(label_filtros, "%s", filtros_texto[filtro_indice]);
    }
    else if (obj == btn2)
    {
      if (!f_nrss)
        f_nrss = true;
      else
        f_nrss = false;

      lv_label_set_text_fmt(label2, "NR=%d", f_nrss);
    }
    else if (obj == btn3)
    {
      agc_wdsp_conf.AGC_mode++;
      if (agc_wdsp_conf.AGC_mode > 5)
        agc_wdsp_conf.AGC_mode = 0;

      agc_wdsp_conf.agc_switch_mode = 1;
      AGC_prep();

      lv_label_set_text_fmt(label3, agc_texto[agc_wdsp_conf.AGC_mode], 0);
    }
    else if (obj == btn4)
    {
      demod_modo = DEMOD_FM;
    }
    else if (obj == btn5)
    {
      if (!screen_update)
        screen_update = true;
      else
        screen_update = false;
    }
    else if (obj == btn_vol)
    {
      // Crear el slider_vol la primera vez
      if (slider_vol == NULL)
      {
        // slider_vol
        slider_vol = lv_slider_create(screen);
        lv_slider_set_range(slider_vol, 0, 100); /* percent of the ES8311 output range */
        lv_slider_set_value(slider_vol, audio_out_get_volume(), LV_ANIM_OFF);
        lv_obj_set_style_bg_color(slider_vol, lv_color_hex(0x202020), LV_PART_MAIN);
        lv_obj_set_style_border_color(slider_vol, lv_color_hex(0x404040), LV_PART_MAIN);

        // Colócalo cerca del botón (debajo, centrado respecto al botón)
        lv_obj_align_to(slider_vol, btn_vol, LV_ALIGN_OUT_TOP_MID, 0, -32);

        // Etiqueta con el valor
        value_label = lv_label_create(screen);
        lv_label_set_text_fmt(value_label, "", lv_slider_get_value(slider_vol) / 0x3F * 100);
        lv_obj_align_to(value_label, slider_vol, LV_ALIGN_OUT_TOP_MID, 60, 0);
        lv_obj_set_style_text_color(value_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

        // Callback para actualizar la etiqueta
        lv_obj_add_event_cb(slider_vol, slider_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
      }
      else
      {
        // Alternar visibilidad
        if (lv_obj_has_flag(slider_vol, LV_OBJ_FLAG_HIDDEN))
        {
          lv_obj_clear_flag(slider_vol, LV_OBJ_FLAG_HIDDEN);
          lv_obj_clear_flag(value_label, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
          lv_obj_add_flag(slider_vol, LV_OBJ_FLAG_HIDDEN);
          lv_obj_add_flag(value_label, LV_OBJ_FLAG_HIDDEN);
        }
      }
    }

    lv_timer_resume(timer_debounce);
  }
}

void dibuja_botones(void)
{
  /* --- Botón Menu--- */
  btn_menu = lv_btn_create(screen);
  lv_obj_set_size(btn_menu, 70, 50);

  lv_obj_align(btn_menu, LV_ALIGN_LEFT_MID, 10, -125); // margen de 10 px desde el borde

  // lv_obj_set_pos(btn_menu, 10, smeter2.header.h + 10);
  lv_obj_add_event_cb(btn_menu, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn_menu, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn_menu, lv_color_hex(0x404040), LV_PART_MAIN);

  label_menu = lv_label_create(btn_menu);
  lv_label_set_text(label_menu, "MENU");
  lv_obj_center(label_menu);

  /* --- Botón Modos--- */
  btn_modos = lv_btn_create(screen);
  lv_obj_set_size(btn_modos, 70, 50);
  lv_obj_align(btn_modos, LV_ALIGN_LEFT_MID, 90, -125); // margen de 10 px desde el borde

  lv_obj_add_event_cb(btn_modos, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn_modos, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn_modos, lv_color_hex(0x404040), LV_PART_MAIN);

  label_modos = lv_label_create(btn_modos);
  lv_label_set_text_fmt(label_modos, "%s", demod_modos_texto[demod_modo]);
  lv_obj_center(label_modos);

  /* --- Botón Filtros--- */
  btn_filtros = lv_btn_create(screen);
  lv_obj_set_size(btn_filtros, 70, 50);
  lv_obj_align(btn_filtros, LV_ALIGN_LEFT_MID, 170, -125); // margen de 10 px desde el borde

  lv_obj_add_event_cb(btn_filtros, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn_filtros, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn_filtros, lv_color_hex(0x404040), LV_PART_MAIN);

  label_filtros = lv_label_create(btn_filtros);
  lv_label_set_text_fmt(label_filtros, "%s", filtros_texto[filtro_indice]);
  lv_obj_center(label_filtros);

  /* --- Botón Step--- */
  btn_step = lv_btn_create(screen);
  lv_obj_set_size(btn_step, 70, 50);
  lv_obj_align(btn_step, LV_ALIGN_LEFT_MID, 250, -125); // margen de 10 px desde el borde

  lv_obj_add_event_cb(btn_step, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn_step, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn_step, lv_color_hex(0x404040), LV_PART_MAIN);

  label_step = lv_label_create(btn_step);
  lv_label_set_text_fmt(label_step, "%d", pasos[pasos_indice]);
  lv_obj_center(label_step);

  return;

  /* --- Botón 2 --- */
  btn2 = lv_btn_create(screen);
  lv_obj_set_size(btn2, 100, 50);
  lv_obj_align(btn2, LV_ALIGN_BOTTOM_LEFT, 120, -10);
  lv_obj_add_event_cb(btn2, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn2, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn2, lv_color_hex(0x404040), LV_PART_MAIN);

  label2 = lv_label_create(btn2);
  lv_label_set_text_fmt(label2, "NR=%d", f_nrss);
  lv_obj_align(btn_modos, LV_ALIGN_BOTTOM_LEFT, 10, -10); // margen de 10 px desde el borde
  lv_obj_center(label2);

  /* --- Botón 4 --- */
  btn4 = lv_btn_create(screen);
  lv_obj_set_size(btn4, 100, 50);
  lv_obj_align(btn4, LV_ALIGN_BOTTOM_LEFT, 340, -10);
  lv_obj_add_event_cb(btn4, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn4, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn3, lv_color_hex(0x404040), LV_PART_MAIN);

  label4 = lv_label_create(btn4);
  lv_label_set_text_fmt(label4, "FM", 0);
  lv_obj_center(label4);

  /* --- Botón 5 --- */
  btn5 = lv_btn_create(screen);
  lv_obj_set_size(btn5, 100, 50);
  lv_obj_align(btn5, LV_ALIGN_BOTTOM_LEFT, 450, -10);
  lv_obj_add_event_cb(btn5, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn5, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn5, lv_color_hex(0x404040), LV_PART_MAIN);

  label5 = lv_label_create(btn5);
  lv_label_set_text_fmt(label5, "", 0);
  lv_obj_center(label5);

  /* --- Botón 6 --- */
  btn6 = lv_btn_create(screen);
  lv_obj_set_size(btn6, 100, 50);
  lv_obj_align(btn6, LV_ALIGN_BOTTOM_LEFT, 560, -10);
  lv_obj_add_event_cb(btn6, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn6, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn6, lv_color_hex(0x404040), LV_PART_MAIN);

  label6 = lv_label_create(btn6);
  lv_label_set_text_fmt(label6, "", 0);
  lv_obj_center(label6);

  /* --- Botón 6 --- */
  btn_vol = lv_btn_create(screen);
  lv_obj_set_size(btn_vol, 100, 50);
  lv_obj_align(btn_vol, LV_ALIGN_BOTTOM_LEFT, 670, -10);
  lv_obj_add_event_cb(btn_vol, btn_event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_set_style_bg_color(btn_vol, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_border_color(btn_vol, lv_color_hex(0x404040), LV_PART_MAIN);

  label_vol = lv_label_create(btn_vol);
  lv_label_set_text_fmt(label_vol, "VOLUME", 0);
  lv_obj_center(label_vol);
}

static void encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  data->enc_diff = s_enc_diff; // entregar Δ acumulado a LVGL
  s_enc_diff = 0;              // limpiar después de reportarlo

  // Si tienes botón físico, cambiarías data->state aquí
  data->state = LV_INDEV_STATE_RELEASED;
}

lv_indev_t *lvgl_encoder_init(void)
{
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER);
  lv_indev_set_read_cb(indev, encoder_read_cb);
  return indev;
}

void tarea_encoder(void *arg)
{
  while (1)
  {
    // Leer delta del encoder
    int32_t diff = s_enc_diff;
    s_enc_diff = 0;

    if (diff != 0)
    {
      currentVFO.Frec += diff * pasos_indice[pasos];
      refresca_VFO();

      rtl_source_set_freq(currentVFO.Frec - FREQ_CONV_OFFSET);
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
  }
}

uint16_t fft_color_map(uint8_t v)
{
  uint8_t r, g, b;

  if (v < 64)
  {
    r = 0;
    g = 0;
    b = v * 4;
  }
  else if (v < 128)
  {
    r = 0;
    g = (v - 64) * 4;
    b = 255;
  }
  else if (v < 192)
  {
    r = (v - 128) * 4;
    g = 255;
    b = 255 - (v - 128) * 4;
  }
  else
  {
    r = 255;
    g = 255 - (v - 192) * 4;
    b = 0;
  }

  return ((r & 0xF8) << 8) |
         ((g & 0xFC) << 3) |
         (b >> 3);
}

void waterfall_update(void)
{
  /*
   * Step the ring back one slot. That slot currently holds the row that is about to
   * fall off the bottom (the oldest one, WATERFALL_HEIGHT steps behind); overwriting
   * it makes it the new top row, and every other row's memory is untouched - no
   * memmove, replacing what used to be a ~260 KB PSRAM copy every frame with two
   * WAVEFORM_WIDTH-pixel writes.
   */
  waterfall_head = (waterfall_head == 0) ? (WATERFALL_HEIGHT - 1) : (waterfall_head - 1);

  uint16_t *row_a = &waterfallbuffer[waterfall_head * WAVEFORM_WIDTH];
  uint16_t *row_b = &waterfallbuffer[(waterfall_head + WATERFALL_HEIGHT) * WAVEFORM_WIDTH];
  for (int x = 0; x < WAVEFORM_WIDTH; x++)
  {
    uint16_t c = fft_color_map((uint8_t)abs(pixelnew[x])); // 0-255 -> RGB565
    row_a[x] = c;
    row_b[x] = c;
  }

  /* Re-point the canvas at the new front of the ring. lv_canvas_set_buffer() only
   * updates the canvas's internal buffer descriptor (no pixel copy), so this is cheap. */
  lv_canvas_set_buffer(waterfall_canvas, row_a, WAVEFORM_WIDTH, WATERFALL_HEIGHT, LV_COLOR_FORMAT_RGB565);
  lv_obj_invalidate(waterfall_canvas);
}

void init_ui()
{
  /* Obtén la pantalla activa */
  screen = lv_scr_act();

  // Padre "limpio"
  lv_obj_remove_style_all(screen);
  lv_obj_set_style_border_width(screen, 0, 0);
  lv_obj_set_style_pad_all(screen, 0, 0);
  lv_obj_set_style_bg_color(screen, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

  lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
  dibuja_botones();

  waveform_canvas = lv_canvas_create(screen);
  waveformbuffer = heap_caps_malloc(
      WAVEFORM_WIDTH * WAVEFORM_HEIGHT * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  memset(waveformbuffer, 0x00, WAVEFORM_WIDTH * WAVEFORM_HEIGHT * sizeof(uint16_t)); // índice 0 (negro)

  // lv_canvas_set_buffer(waveform_canvas, waveformbuffer, WAVEFORM_WIDTH, WAVEFORM_HEIGHT, LV_COLOR_FORMAT_I4);

  lv_canvas_set_buffer(waveform_canvas, waveformbuffer, CANVAS_W, CANVAS_H, LV_COLOR_FORMAT_RGB565);

  waterfall_canvas = lv_canvas_create(screen);
  /* Double-height ring buffer (see the field comment in ui_priv.h): was
   * WAVEFORM_WIDTH * WAVEFORM_HEIGHT (192 rows' worth, more than needed by mistake -
   * WATERFALL_HEIGHT is 128 - but harmless since it only over-allocated). Now sized
   * and named for what it actually holds: 2 * WATERFALL_HEIGHT rows. */
  waterfallbuffer = heap_caps_malloc(
      2u * WAVEFORM_WIDTH * WATERFALL_HEIGHT * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  memset(waterfallbuffer, 0x00, 2u * WAVEFORM_WIDTH * WATERFALL_HEIGHT * sizeof(uint16_t)); // índice 0 (negro)
  waterfall_head = 0;
  lv_canvas_set_buffer(waterfall_canvas, waterfallbuffer, WAVEFORM_WIDTH, WATERFALL_HEIGHT, LV_COLOR_FORMAT_RGB565);

  // POSICION DE LOS CANVAS DE WATERFALL Y ESPECTRO!!!

  lv_obj_set_pos(waveform_canvas, 0, 600 - WATERFALL_HEIGHT - WAVEFORM_HEIGHT);
  lv_obj_align_to(waterfall_canvas, waveform_canvas, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

  // Borde y fondo
  // lv_obj_set_style_bg_color(waveform_canvas, lv_color_hex(0x000000), 0);
  // lv_obj_set_style_bg_opa(waveform_canvas, LV_OPA_COVER, 0);
  // lv_obj_set_style_border_width(waveform_canvas, 2, 0);
  // lv_obj_set_style_border_color(waveform_canvas, lv_color_hex(0x999999), 0);
  // lv_obj_set_style_radius(waveform_canvas, 6, 0);
  // lv_obj_set_style_pad_all(waveform_canvas, 0, 0);

  // // Para una “aura” visible opcional:
  // lv_obj_set_style_outline_width(waveform_canvas, 2, 0);
  // lv_obj_set_style_outline_color(waveform_canvas, lv_color_hex(0x00FFA0), 0);

  lv_obj_add_flag(waterfall_canvas, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(waterfall_canvas, btn_event_cb, LV_EVENT_CLICKED, NULL);

  // ✅ Forzar actualización
  lv_obj_invalidate(waveform_canvas);

  // Crear un label
  freq_label = lv_label_create(screen);

  // Establecer estilo de fuente
  static lv_style_t style_freq;
  lv_style_init(&style_freq);
  lv_style_set_text_font(&style_freq, &lv_font_montserrat_46);
  lv_obj_add_style(freq_label, &style_freq, LV_STATE_DEFAULT);

  // Color verde
  lv_style_set_text_color(&style_freq, lv_color_hex(0x00FF00));

  // Posición opcional
  lv_obj_align(freq_label, LV_ALIGN_BOTTOM_RIGHT, -10, -550);

  lv_obj_add_flag(freq_label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(freq_label, freq_label_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_style_text_decor(freq_label, LV_TEXT_DECOR_UNDERLINE, LV_STATE_PRESSED);

  refresca_VFO();

  /* TIMERS
   */

  timer_pantalla = lv_timer_create(timer_dibuja_pantalla, 33, NULL);
  // timer_cpu = lv_timer_create(timer_uso_cpu, 1000, NULL);
  timer_smeter = lv_timer_create(timer_smeter_update, 33, NULL);
  // timer debounce 
  timer_debounce = lv_timer_create(desbloquear_cb, 300, NULL);

  /* TIMERS
   */

  box_pasabanda = lv_obj_create(lv_screen_active());

  lv_obj_set_style_bg_opa(box_pasabanda, 80, LV_PART_MAIN); // 80/255 ≈ 30% opaco
  lv_obj_set_style_bg_color(box_pasabanda, lv_color_hex(0x000088), LV_PART_MAIN);

  // Quitar borde
  lv_obj_set_style_border_width(box_pasabanda, 0, LV_PART_MAIN);

  // Opcional: quitar sombra si no la quieres
  lv_obj_set_style_shadow_width(box_pasabanda, 0, LV_PART_MAIN);

  init_smeter();
  inicia_smeter_ui();
  dibuja_pasabanda();
  indicadores_create(screen);
  refresca_indicadores();
}

void smeter_set_dbm(float dbm)
{
  if (dbm < SMETER_DBM_MIN)
    dbm = SMETER_DBM_MIN;

  if (dbm > SMETER_DBM_MAX)
    dbm = SMETER_DBM_MAX;

  /* ESTA ES LA FÓRMULA CORRECTA */
  float ratio = (dbm - SMETER_DBM_MIN) /
                (SMETER_DBM_MAX - SMETER_DBM_MIN);

  /* Mapear dBm → ángulo */
  float angle_f =
      SMETER_ANGLE_MIN +
      ratio * (SMETER_ANGLE_MAX - SMETER_ANGLE_MIN);

  int angle_deg = (int)angle_f;

  /* lv_line_set_points() always invalidates the needle area (335x145 px here), even if the
   * points did not change. Skip it while the needle stays on the same degree. */
  static int last_angle_deg = -100000;
  if (angle_deg == last_angle_deg)
    return;
  last_angle_deg = angle_deg;

  int32_t s = lv_trigo_sin(angle_deg);
  int32_t c = lv_trigo_cos(angle_deg);

  int32_t dx = (c * SMETER_NEEDLE_LEN) >> LV_TRIGO_SHIFT;
  int32_t dy = (s * SMETER_NEEDLE_LEN) >> LV_TRIGO_SHIFT;

  smeter.pts[0].x = SMETER_PIVOT_X;
  smeter.pts[0].y = SMETER_PIVOT_Y;

  smeter.pts[1].x = SMETER_PIVOT_X - dx;
  smeter.pts[1].y = SMETER_PIVOT_Y - dy;

  lv_line_set_points(smeter.needle, smeter.pts, 2);

  // if (contador_printf++ > 25)
  // {
  //   printf("%f - angle: %f aguja: %d,%d -> %d,%d \n", dbm, angle_f, SMETER_PIVOT_X, SMETER_PIVOT_Y, smeter.pts[1].x, smeter.pts[1].y);
  //   contador_printf = 0;
  // }
}

void inicia_smeter_ui(void)
{
  /* 1. Crear contenedor del smeter */
  meter_cont = lv_obj_create(screen);
  lv_obj_set_size(meter_cont, smeter2.header.w, smeter2.header.h);
  lv_obj_set_style_pad_all(meter_cont, 0, 0);
  lv_obj_set_style_border_width(meter_cont, 0, 0);

  /* Activar CLIPPING */
  lv_obj_set_scroll_dir(meter_cont, LV_DIR_NONE); // evita scroll
  lv_obj_set_scrollbar_mode(meter_cont, LV_SCROLLBAR_MODE_OFF);

  /* 2. Crear imagen del smeter dentro del contenedor */
  meter_img = lv_img_create(meter_cont);
  lv_img_set_src(meter_img, &smeter2);
  lv_obj_set_pos(meter_img, 0, 0);

  /* 3. Crear aguja como línea */
  smeter.needle = lv_line_create(meter_cont);
  lv_obj_set_size(smeter.needle, smeter2.header.w, smeter2.header.h + 20.);

  /* Estilo de la aguja */
  lv_obj_set_style_line_width(smeter.needle, 4, 0);
  lv_obj_set_style_line_color(smeter.needle, lv_color_hex(0xff0000), 0);
  lv_obj_set_style_line_rounded(smeter.needle, true, 0);

  lv_obj_set_pos(smeter.needle, 0, 0);

  lv_obj_set_style_border_width(meter_cont, 2, 0);
  lv_obj_set_style_border_color(meter_cont, lv_color_hex(0x999999), 0);
  lv_obj_set_style_radius(meter_cont, 6, 0);
  lv_obj_set_style_pad_all(meter_cont, 0, 0);
}

void dibuja_pasabanda(void)
{

  int margen_alto = currentVFO.f_alta * SAMPLE_BUFFER_SIZE / SAMPLE_RATE;

  if (lvgl_port_lock(0))
  {

    if (demod_modo == DEMOD_AM || demod_modo == DEMOD_SAM)
    {
      lv_obj_set_size(box_pasabanda, 2 * margen_alto, H);
      lv_obj_set_pos(box_pasabanda, W - W / 4 - margen_alto, 600 - WATERFALL_HEIGHT - WAVEFORM_HEIGHT);
    }
    else if (demod_modo == DEMOD_USB || demod_modo == DEMOD_SAMU)
    {
      lv_obj_set_size(box_pasabanda, margen_alto, H);
      lv_obj_set_pos(box_pasabanda, W - W / 4, 600 - WATERFALL_HEIGHT - WAVEFORM_HEIGHT);
    }
    else if (demod_modo == DEMOD_LSB || demod_modo == DEMOD_SAML)
    {
      lv_obj_set_size(box_pasabanda, margen_alto, H);
      lv_obj_set_pos(box_pasabanda, W - W / 4 - margen_alto, 600 - WATERFALL_HEIGHT - WAVEFORM_HEIGHT);
    }
    else if (demod_modo == DEMOD_FM)
    {
      lv_obj_set_size(box_pasabanda, 0, 0);
      lv_obj_set_pos(box_pasabanda, W, 600 - WATERFALL_HEIGHT - WAVEFORM_HEIGHT);
    }

    lvgl_port_unlock();
  }
}

// ---------- Objetos (static fuera de funciones) ----------
static lv_obj_t *indicadores[12] = {0};

// ---------- Estilos (static fuera de funciones) ----------
static lv_style_t st_base;
static lv_style_t st_red;
static lv_style_t st_green;
static bool styles_inited = false;

// Usaremos LV_STATE_USER_1 = "verde"
#define IND_VERDE_STATE LV_STATE_USER_1

static void init_styles_once(void)
{
  if (styles_inited)
    return;
  styles_inited = true;

  lv_style_init(&st_base);
  lv_style_set_bg_opa(&st_base, LV_OPA_COVER);
  lv_style_set_bg_color(&st_base, lv_color_black());
  lv_style_set_border_width(&st_base, 2);
  lv_style_set_radius(&st_base, 4);
  lv_style_set_pad_all(&st_base, 8);
  lv_style_set_text_align(&st_base, LV_TEXT_ALIGN_CENTER);

  // Estilo por defecto: ROJO (estado 0)
  lv_style_init(&st_red);
  lv_style_set_text_color(&st_red, lv_color_hex(0xFF0000));
  lv_style_set_border_color(&st_red, lv_color_hex(0xFF0000));

  // Estilo cuando esté en estado VERDE (USER_1)
  lv_style_init(&st_green);
  lv_style_set_text_color(&st_green, lv_color_hex(0x00FF00));
  lv_style_set_border_color(&st_green, lv_color_hex(0x00FF00));
}

// Crea los 9 indicadores UNA SOLA VEZ
void indicadores_create(lv_obj_t *parent)
{
  init_styles_once();

  for (int i = 0; i < 12; i++)
  {

    indicadores[i] = lv_label_create(parent);

    lv_obj_add_style(indicadores[i], &st_base, LV_PART_MAIN);
    lv_obj_add_style(indicadores[i], &st_red, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_style(indicadores[i], &st_green, LV_PART_MAIN | IND_VERDE_STATE);

    // Pon aquí su geometría como tú quieras (ejemplo)
    if (i < 5)
    {
      lv_obj_set_pos(indicadores[i], 350 + i * 70, 0);
      lv_obj_set_width(indicadores[i], 60);
    }
    else if (i >= 5 && i < 10)
    {
      lv_obj_set_pos(indicadores[i], 350 + (i - 5) * 70, 50);
      lv_obj_set_width(indicadores[i], 60);
    }
    else if (i >= 10)
    {
      lv_obj_set_pos(indicadores[i], 350 + (i - 10) * 110, 100);
      lv_obj_set_width(indicadores[i], 100);
    }

    lv_label_set_text(indicadores[i], "X");
    lv_obj_clear_state(indicadores[i], IND_VERDE_STATE); // arranca en rojo
  }
}

// Actualiza texto y color SIN recrear y SIN tocar estilos
void indicador_update(int idx, const char *texto, bool verde)
{
  if (idx < 0 || idx >= 12)
    return;
  lv_obj_t *lbl = indicadores[idx];
  if (!lbl)
    return;

  // Cambia texto sólo si cambia (evita trabajo)
  const char *cur = lv_label_get_text(lbl);
  if (!cur || strcmp(cur, texto) != 0)
  {
    lv_label_set_text(lbl, texto);
  }

  // Cambia estado (rojo por defecto, verde con USER_1)
  if (verde)
    lv_obj_add_state(lbl, IND_VERDE_STATE);
  else
    lv_obj_clear_state(lbl, IND_VERDE_STATE);
}

void refresca_indicadores(void)
{
  indicador_update(0, "A", true);

  indicador_update(1, "RX", true);

  indicador_update(2, "AM", (demod_modo == DEMOD_AM) ? 1 : 0);
  indicador_update(3, "SAM", (demod_modo == DEMOD_SAM || demod_modo == DEMOD_SAMU || demod_modo == DEMOD_SAML) ? 1 : 0);
  indicador_update(4, "USB", (demod_modo == DEMOD_USB) ? 1 : 0);

  indicador_update(5, "B", true);

  indicador_update(6, "LSB", (demod_modo == DEMOD_LSB) ? 1 : 0);

  indicador_update(7, "DNR", (f_nrss) ? 1 : 0);
  indicador_update(8, "NFM", (demod_modo == DEMOD_FM) ? 1 : 0);
  indicador_update(9, "WFM", (demod_modo == DEMOD_FM) ? 1 : 0);

  indicador_update(10, filtros_texto[filtro_indice], true);
  indicador_update(11, agc_texto[agc_wdsp_conf.AGC_mode], true);
}