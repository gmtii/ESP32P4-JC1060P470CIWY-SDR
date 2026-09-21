set(INCLUDE_DIR ./include)

idf_component_register(
    SRCS 
    main.c
    i2s_driver.c
    nau8822.c
    sdr.c
    agc.c
    nr.c
    sam.c
    ui.c
    sdr_math.c
    dsp_example.c
    nr_ss.c
    smeter.c
    rtl_dsp.c
    rtl_source.c
    uart_commands.c
    menu.c
    INCLUDE_DIRS .  ${INCLUDE_DIR}
    ) 

idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
target_compile_options(
    ${LVGL_LIB} 
    PRIVATE
        -DLV_LVGL_H_INCLUDE_SIMPLE
)

