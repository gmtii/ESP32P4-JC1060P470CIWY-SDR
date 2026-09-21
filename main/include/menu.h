#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    extern lv_obj_t *freq_label;

    void ui_create_control_panel(void);
    void ui_create_menu_lvgl92(void);
    void freq_label_event_cb(lv_event_t *e);

#ifdef __cplusplus
}
#endif