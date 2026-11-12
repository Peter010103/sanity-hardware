#include "sdkconfig.h"
#include "hx711.h"
#ifdef CONFIG_SANITY_HX711
#include "hx711_core.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>

/* Readiness is POLLED, not a conversion-midpoint timestamp. Only individual
 * short clock-HIGH pulses mask interrupts; never polling, reset or networking.
 * A128 uses exactly25 pulses. Keep the noinline loop/GPIO functions in IRAM. */
static portMUX_TYPE pulse_lock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t samples;
static uint64_t acquisition_boot, receiver_boot;
static uint32_t receiver_boot_count, queue_drops;
static bool gpio_ok;
static void converter_reset(void) {
    gpio_set_level(CONFIG_SANITY_HX711_SCK,1);
    esp_rom_delay_us(80);
    gpio_set_level(CONFIG_SANITY_HX711_SCK,0);
}
static bool IRAM_ATTR __attribute__((noinline)) read_a128(int32_t *raw, int64_t *ready_us, uint32_t *max_high_us) {
    if (gpio_get_level(CONFIG_SANITY_HX711_DOUT)) return false;
    *ready_us=esp_timer_get_time();
    uint32_t bits=0;
    for (int i=0;i<25;++i) {
        portENTER_CRITICAL(&pulse_lock);
        uint32_t before=esp_cpu_get_cycle_count();
        gpio_set_level(CONFIG_SANITY_HX711_SCK,1);
        esp_rom_delay_us(2);
        int value=gpio_get_level(CONFIG_SANITY_HX711_DOUT);
        gpio_set_level(CONFIG_SANITY_HX711_SCK,0);
        uint32_t cycles=esp_cpu_get_cycle_count()-before;
        uint32_t high=(cycles+CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ-1)/CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
        portEXIT_CRITICAL(&pulse_lock);
        if(high>*max_high_us)*max_high_us=high;
        if(i<24)bits=(bits<<1)|(uint32_t)value;
        esp_rom_delay_us(2);
    }
    *raw=(bits&0x800000u)?(int32_t)((int64_t)bits-0x1000000LL):(int32_t)bits;
    return true;
}
static void enqueue(const struct sz_hx_sample *sample) {
    /* Single producer. Drop oldest rather than wait or replay a long backlog.
     * A simultaneous consumer may already have freed the full queue. */
    if(xQueueSend(samples,sample,0)==pdTRUE)return;
    struct sz_hx_sample old;
    if(xQueueReceive(samples,&old,0)==pdTRUE)
        __atomic_add_fetch(&queue_drops,1,__ATOMIC_RELAXED);
    if(xQueueSend(samples,sample,0)!=pdTRUE)
        __atomic_add_fetch(&queue_drops,1,__ATOMIC_RELAXED);
}
static void acquisition(void *unused) {
    (void)unused;
    struct sz_hx_state state;
    if(gpio_ok)converter_reset();
    sz_hx_init(&state,esp_timer_get_time());
    if(!gpio_ok){state.resets=0;state.reset_us=0;state.discard=0;state.settling_until_us=0;}
    int64_t lower=0;
    for(;;) {
        int32_t raw=0;int64_t ready=0,shift_end=0;uint32_t high=0;
        bool have=false,stuck=false;
        if(gpio_ok) {
            /* Timestamp BEFORE the high observation: a conservative edge lower bound. */
            int64_t poll_before=esp_timer_get_time();
            if(gpio_get_level(CONFIG_SANITY_HX711_DOUT))lower=poll_before;
            else {
                have=read_a128(&raw,&ready,&high);
                if(have){shift_end=esp_timer_get_time();stuck=!gpio_get_level(CONFIG_SANITY_HX711_DOUT);}
            }
        }
        bool reset=false;struct sz_hx_sample sample;
        if(sz_hx_step(&state,have,raw,ready,lower,shift_end,high,stuck,
                      esp_timer_get_time(),&sample,&reset)) {
            if(reset){
                converter_reset();sz_hx_reset_done(&state,esp_timer_get_time());
                sample.reset_us=state.reset_us;sample.acquired_us=esp_timer_get_time();
            }
            enqueue(&sample);
        }
        if(have)lower=0;
        vTaskDelay(1); /* Measured source intervals; no80/90Hz cadence forced. */
    }
}
static void telemetry(void *unused) {
    (void)unused;
    struct sockaddr_in destination={.sin_family=AF_INET,.sin_port=htons(CONFIG_SANITY_HX711_PORT)};
    if(inet_pton(AF_INET,CONFIG_SANITY_HX711_HOST,&destination.sin_addr)!=1){
        ESP_LOGE("hx711","Invalid telemetry IPv4; raw acquisition continues");vTaskDelete(NULL);return;
    }
    int fd=-1;int64_t retry=0;uint32_t drops=0;
    for(;;){
        struct sz_hx_sample sample;
        if(xQueueReceive(samples,&sample,pdMS_TO_TICKS(100))!=pdTRUE)continue;
        int64_t now=esp_timer_get_time();
        /* Do not release stale local backlog even after a stalled network task. */
        if(now-sample.acquired_us>100000){
            __atomic_add_fetch(&queue_drops,1,__ATOMIC_RELAXED);continue;
        }
        if(fd<0 && now>=retry){
            fd=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
            if(fd>=0 && fcntl(fd,F_SETFL,O_NONBLOCK)<0){close(fd);fd=-1;}
            retry=now+1000000;
        }
        if(fd<0){++drops;continue;} /* No wait/retry loop for an individual packet. */
        char packet[768];
        /* read_us retains its legacy meaning: pre-format/UDP-dispatch timestamp,
         * not the last SCK edge. acquired_us/shift_end_us separate the stages. */
        int length=sz_hx_format(packet,sizeof(packet),&sample,CONFIG_SANITY_DEVICE_ID,
            acquisition_boot,receiver_boot,receiver_boot_count,esp_timer_get_time(),
            drops,__atomic_load_n(&queue_drops,__ATOMIC_RELAXED));
        if(length<0 || (size_t)length>=sizeof(packet)){++drops;continue;}
        if(sendto(fd,packet,(size_t)length,0,(struct sockaddr *)&destination,sizeof(destination))!=length){
            ++drops;
            if(errno==EBADF || errno==ENOTSOCK){close(fd);fd=-1;}
        }
    }
}
void sz_hx711_start(uint64_t boot, uint32_t count) {
#ifdef CONFIG_PM_ENABLE
    ESP_LOGE("hx711","Fixed CPU frequency required; sensor disabled, receiver continues");return;
#endif
    int data=CONFIG_SANITY_HX711_DOUT,clock=CONFIG_SANITY_HX711_SCK;
    if(data!=1 || clock!=2 || data==CONFIG_SANITY_UART_RX || data==CONFIG_SANITY_UART_TX ||
       clock==CONFIG_SANITY_UART_RX || clock==CONFIG_SANITY_UART_TX){
        ESP_LOGE("hx711","Use free GPIO1/D0 and GPIO2/D1; sensor disabled");return;
    }
    if(samples){ESP_LOGE("hx711","Already started");return;}
    receiver_boot=boot;receiver_boot_count=count;
    acquisition_boot=((uint64_t)esp_random()<<32)|esp_random();
    samples=xQueueCreate(SZ_HX_QUEUE_SIZE,sizeof(struct sz_hx_sample));
    if(!samples){ESP_LOGE("hx711","No sensor queue; receiver continues");return;}
    gpio_config_t out={.pin_bit_mask=1ULL<<clock,.mode=GPIO_MODE_OUTPUT};
    gpio_config_t in={.pin_bit_mask=1ULL<<data,.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE};
    gpio_ok=gpio_config(&out)==ESP_OK && gpio_set_level(clock,0)==ESP_OK && gpio_config(&in)==ESP_OK;
    if(!gpio_ok)ESP_LOGE("hx711","GPIO init failed; only missing-status heartbeats");
    if(xTaskCreate(acquisition,"hx711_read",4096,NULL,2,NULL)!=pdPASS){
        vQueueDelete(samples);samples=NULL;ESP_LOGE("hx711","Cannot start acquisition; receiver continues");return;
    }
    if(xTaskCreate(telemetry,"hx711_udp",4096,NULL,1,NULL)!=pdPASS)
        ESP_LOGE("hx711","Cannot start telemetry; acquisition/receiver continue");
}
#else
void sz_hx711_start(uint64_t boot, uint32_t count){(void)boot;(void)count;}
#endif
