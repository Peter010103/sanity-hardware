#ifndef HX711_NATIVE_SHIM_H
#define HX711_NATIVE_SHIM_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#define CONFIG_SANITY_HX711 1
#define CONFIG_SANITY_HX711_SCK 2
#define CONFIG_SANITY_HX711_DOUT 1
#define CONFIG_SANITY_UART_RX 3
#define CONFIG_SANITY_UART_TX 4
#define CONFIG_SANITY_HX711_PORT 7460
#define CONFIG_SANITY_HX711_HOST "127.0.0.1"
#define CONFIG_SANITY_DEVICE_ID 9
#define CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ 240
#define IRAM_ATTR
#define ESP_OK 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_INPUT 2
#define GPIO_PULLUP_ENABLE 1
#define pdTRUE 1
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
#define portMUX_TYPE pthread_mutex_t
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define portENTER_CRITICAL(p) pthread_mutex_lock(p)
#define portEXIT_CRITICAL(p) pthread_mutex_unlock(p)
#define ESP_LOGE(tag, ...) do {fprintf(stderr,"%s: ",tag);fprintf(stderr,__VA_ARGS__);fprintf(stderr,"\n");}while(0)
typedef struct {uint64_t pin_bit_mask;int mode,pull_up_en;} gpio_config_t;
typedef struct fake_queue *QueueHandle_t;
int gpio_config(const gpio_config_t *config);
int gpio_set_level(int pin,int value);
int gpio_get_level(int pin);
int64_t esp_timer_get_time(void);
uint32_t esp_cpu_get_cycle_count(void);
uint32_t esp_random(void);
void esp_rom_delay_us(unsigned us);
void vTaskDelay(unsigned ticks);
void vTaskDelete(void *task);
int xTaskCreate(void (*run)(void*),const char *name,unsigned stack,void *arg,unsigned priority,void *handle);
QueueHandle_t xQueueCreate(unsigned count,unsigned size);
int xQueueSend(QueueHandle_t q,const void *item,unsigned wait);
int xQueueReceive(QueueHandle_t q,void *item,unsigned wait);
void vQueueDelete(QueueHandle_t q);
int hx_socket(int domain,int type,int protocol);
int hx_fcntl(int fd,int command,...);
int hx_close(int fd);
ssize_t hx_sendto(int fd,const void *data,size_t length,int flags,const struct sockaddr *to,socklen_t size);
#define socket hx_socket
#define fcntl hx_fcntl
#define close hx_close
#define sendto hx_sendto
#endif
