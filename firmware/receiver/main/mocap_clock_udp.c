#include "sdkconfig.h"
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
#include "mocap_clock_udp.h"
#include "mocap_state.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

static uint8_t clock_device;
static uint64_t (*clock_epoch)(void);

static void clock_task(void *unused) {
    (void)unused;
    for (;;) {
        int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        struct sockaddr_in local = {.sin_family = AF_INET,
            .sin_port = htons(CONFIG_SANITY_MOCAP_CLOCK_UDP_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY)};
        struct timeval rx_timeout = {.tv_sec = 1};
        struct timeval tx_timeout = {.tv_usec = 10000};
        if (fd < 0 || setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout)) < 0 ||
            setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tx_timeout, sizeof(tx_timeout)) < 0 ||
            bind(fd, (struct sockaddr *)&local, sizeof(local)) < 0) {
            if (fd >= 0) close(fd);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI("mocap_clock", "Observation clock UDP port %d", CONFIG_SANITY_MOCAP_CLOCK_UDP_PORT);
        uint64_t last_reply = 0;
        for (;;) {
            /* Bound work even if malformed datagrams arrive continuously. */
            vTaskDelay(pdMS_TO_TICKS(1));
            /* One extra byte ensures oversized datagrams are rejected. */
            uint8_t request[SZ_MOCAP_CLOCK_REQUEST_SIZE + 1];
            struct sockaddr_in peer;
            socklen_t peer_size = sizeof(peer);
            int n = recvfrom(fd, request, sizeof(request), 0, (struct sockaddr *)&peer, &peer_size);
            uint64_t received = esp_timer_get_time();
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
                break;
            }
            uint64_t nonce, host;
            if (!sz_mocap_clock_request(request, (size_t)n, clock_device, &nonce, &host)) continue;
            uint64_t epoch = clock_epoch();
            if (!epoch || (last_reply && received - last_reply < 10000)) continue;
            uint8_t reply[SZ_MOCAP_CLOCK_REPLY_SIZE];
            sz_mocap_clock_reply(clock_device, epoch, nonce, host, received, esp_timer_get_time(), reply);
            sendto(fd, reply, sizeof(reply), 0, (struct sockaddr *)&peer, peer_size);
            last_reply = received;
        }
        close(fd);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sz_mocap_clock_udp_start(uint8_t device, uint64_t (*current_epoch)(void)) {
    clock_device = device;
    clock_epoch = current_epoch;
    static StaticTask_t task_buffer;
    static StackType_t task_stack[4096 / sizeof(StackType_t)];
    /* Wi-Fi and TCP/IP retain their higher priorities. No UART/ESKF work here. */
    if (!current_epoch || !xTaskCreateStaticPinnedToCore(clock_task, "mocap_clock", 4096, NULL, 14,
                                                       task_stack, &task_buffer, 0)) abort();
}
#endif
