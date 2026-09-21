#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"

    void spectrum(void);
    void init_ui(void);
    void nuevo(void);
    void dibuja_botones(void);
    lv_obj_t *smeter_create(lv_obj_t *parent, int size);
    lv_indev_t *lvgl_encoder_init(void);
    void tarea_encoder(void *arg);
    void waterfall_update(void);
    void waterfall_scroll_down(uint16_t *buf);
    void dibuja_pasabanda(void);
    void inicia_smeter_ui(void);
    void smeter_set_dbm(float dbm);
    void crear_label_con_estilo(lv_obj_t *parent, char *texto, int x, int y, int ancho, bool color_verde);
    void refresca_VFO(void);
    void timer_dibuja_pantalla(lv_timer_t *timer);
    void timer_smeter_update(lv_timer_t *timer);
    void timer_uso_cpu(lv_timer_t *timer);
    void refresca_indicadores(void);
    void indicadores_create(lv_obj_t *parent);
    void indicador_update(int idx, const char *texto, bool verde);
    static void init_styles_once(void);

    extern lv_timer_t *timer_pantalla;
    extern lv_timer_t *timer_cpu;
    extern lv_timer_t *timer_smeter;

#ifdef __cplusplus
}
#endif