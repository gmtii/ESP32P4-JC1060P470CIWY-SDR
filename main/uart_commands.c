#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#include "uart_commands.h"
#include "msi001.h"
#include "nau8822.h"

#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

static const char *TAG = "UART_CMD";

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);
}

void uart_command_handler(char *cmd) {
    if (strcmp(cmd, "+") == 0) {
        ESP_LOGI(TAG, "Comando recibido: encender LED");
    }
    else if (strcmp(cmd, "-") == 0) {
        ESP_LOGI(TAG, "Comando recibido: apagar LED");
    }
    else {
        ESP_LOGW(TAG, "Comando desconocido: %s", cmd);
    }
}

void uart_command_loop(void *arg) {
    uint8_t data[BUF_SIZE];
    char command[128];
    int cmd_index = 0;

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, 1, pdMS_TO_TICKS(20));

        if (len > 0) {
            char c = data[0];

            if (c == '\n' || c == '\r') {
                if (cmd_index > 0) {
                    command[cmd_index] = '\0';
                    uart_command_handler(command);
                    cmd_index = 0;
                }
            } else {
                if (cmd_index < sizeof(command) - 1) {
                    command[cmd_index++] = c;
                }
            }
        }
    }
}