#include "passthrough.h"
#include "sdkconfig.h"
#include <stdlib.h>

#ifdef CONFIG_SANITY_USB_PASSTHROUGH
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG) || \
    defined(CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG) || defined(CONFIG_ESP_CONSOLE_USB_CDC)
#error "USB passthrough needs USB console disabled; use a board UART defaults profile"
#endif
#include "bridge_stream.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BRIDGE_UART UART_NUM_2

static void mode_led(bool on) {
#if CONFIG_SANITY_PASSTHROUGH_LED_GPIO >= 0
    gpio_set_level(CONFIG_SANITY_PASSTHROUGH_LED_GPIO, on ? 0 : 1);
#else
    (void)on;
#endif
}

bool sz_usb_passthrough_selected(void) {
    bool selected = false;
#ifdef CONFIG_SANITY_PASSTHROUGH_DEFAULT_ON
    selected = true;
#endif
#if CONFIG_SANITY_PASSTHROUGH_LED_GPIO >= 0
    gpio_config_t led = {.pin_bit_mask = 1ULL << CONFIG_SANITY_PASSTHROUGH_LED_GPIO,
                         .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&led));
#endif
    gpio_config_t button = {.pin_bit_mask = 1ULL << CONFIG_SANITY_PASSTHROUGH_BUTTON_GPIO,
                           .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&button));
    int64_t start = esp_timer_get_time(), pressed = 0;
    while (esp_timer_get_time() - start < CONFIG_SANITY_PASSTHROUGH_WINDOW_MS * 1000LL) {
        int64_t now = esp_timer_get_time();
        mode_led(((now - start) / 150000) % 2 == 0);
        if (gpio_get_level(CONFIG_SANITY_PASSTHROUGH_BUTTON_GPIO) == 0) {
            if (!pressed)
                pressed = now;
            if (now - pressed >= 80000) {
                selected = !selected;
                break;
            }
        } else {
            pressed = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    mode_led(selected);
    return selected;
}

static int usb_read(void *buffer, size_t size) {
    return usb_serial_jtag_read_bytes(buffer, size, 0);
}

static int usb_write(const void *buffer, size_t size) {
    return usb_serial_jtag_write_bytes(buffer, size, 0);
}

static int fc_read(void *buffer, size_t size) {
    return uart_read_bytes(BRIDGE_UART, buffer, size, 0);
}

static int fc_write(const void *buffer, size_t size) {
    return uart_write_bytes(BRIDGE_UART, buffer, size);
}

void sz_usb_passthrough_run(void) {
    uart_config_t config = {.baud_rate = CONFIG_SANITY_PASSTHROUGH_BAUD,
                            .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                            .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART, CONFIG_SANITY_UART_TX, CONFIG_SANITY_UART_RX,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_line_inverse(BRIDGE_UART, 0));
    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART, 16384, 4096, 0, NULL, 0));
    usb_serial_jtag_driver_config_t usb = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb.tx_buffer_size = 8192;
    usb.rx_buffer_size = 4096;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    struct sz_bridge_stream to_fc = {0}, to_usb = {0};
    for (;;) {
        if (sz_bridge_pump(&to_fc, usb_read, fc_write) < 0 ||
            sz_bridge_pump(&to_usb, fc_read, usb_write) < 0)
            for (;;) {
                mode_led((esp_timer_get_time() / 200000) % 2 == 0);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#else
bool sz_usb_passthrough_selected(void) { return false; }
void sz_usb_passthrough_run(void) { abort(); }
#endif
