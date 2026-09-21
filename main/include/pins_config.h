#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#define LCD_H_RES 1024
#define LCD_V_RES 600

#define LCD_RST 27
#define LCD_LED 23

#define TP_I2C_SDA 7
#define TP_I2C_SCL 8
#define TP_RST 22
#define TP_INT 21

/* vfo encoder knob*/
#define GPIO_KNOB_A (GPIO_NUM_47)
#define GPIO_KNOB_B (GPIO_NUM_46)
#define KNOB_NUM (GPIO_NUM_45)

    /* i2s codec*/

#define I2S_NUM (I2S_NUM_0)

#define I2S_MCK_IO (GPIO_NUM_1)
#define I2S_WS_IO (GPIO_NUM_2)
#define I2S_BCK_IO (GPIO_NUM_3)
#define I2S_DO_IO (GPIO_NUM_4)
#define I2S_DI_IO (GPIO_NUM_5)

    /* nau8828 */

#define NAU8822_CS_PIN (GPIO_NUM_20)
#define NAU8822_SPI_CLK_PIN (GPIO_NUM_32)
#define NAU8822_SPI_DAT_PIN (GPIO_NUM_33)

/* msi001 */
#define msi001_dat_pin (GPIO_NUM_33)
#define msi001_clk_pin (GPIO_NUM_32)
#define msi001_cs_pin (GPIO_NUM_45)

#define I2C_NUM (I2C_NUM_0)
#define I2C_SCL_IO (GPIO_NUM_8)
#define I2C_SDA_IO (GPIO_NUM_7)

#ifdef __cplusplus
}
#endif