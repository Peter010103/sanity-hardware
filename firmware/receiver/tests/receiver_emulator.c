/* Real Zenoh-Pico + the firmware's exact control/encoding code. No hardware I/O. */
#define _POSIX_C_SOURCE 200809L
#include "control.h"
#include "zenoh-pico.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
static struct sz_state state;
static bool unified = false;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static uint32_t millis(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint32_t)((uint64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000);
}
static void receive(z_loaned_sample_t *sample, void *context) {
    (void)context;
    uint8_t data[SZ_COMMAND_SIZE];
    struct sz_command command;
    const z_loaned_bytes_t *bytes = z_sample_payload(sample);
    if (z_bytes_len(bytes) != sizeof(data))
        return;
    z_bytes_reader_t reader = z_bytes_get_reader(bytes);
    if (z_bytes_reader_read(&reader, data, sizeof(data)) != sizeof(data) ||
        !sz_decode(data, sizeof(data), &command) || !sz_calibrate(&command,7800))
        return;
    pthread_mutex_lock(&mutex);
    if (unified) sz_accept_msp(&state, &command, millis(), true);
    else sz_accept(&state, &command, millis());
    pthread_mutex_unlock(&mutex);
}
int main(int argc, char **argv) {
    if (argc != 4)
        return 2;
    int mode = atoi(argv[1]);
    unified = mode == 3;
    uint32_t duration = (uint32_t)(atof(argv[3]) * 1000);
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t epoch = ((uint64_t)getpid() << 32) | (uint32_t)t.tv_nsec;
    sz_init(&state, 1, unified ? SZ_CTBR : (uint8_t)mode, epoch);
    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, "client");
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, argv[2]);
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MULTICAST_SCOUTING_KEY, "false");
    z_owned_session_t session;
    if (z_open(&session, z_config_move(&config), NULL) < 0)
        return 3;
    z_view_keyexpr_t command_key, status_key;
    z_view_keyexpr_from_str(&command_key, unified ? "sanity/01/command/*" : mode == SZ_CTBR ? "sanity/01/command/ctbr"
                                                          : "sanity/01/command/direct_thrust");
    z_view_keyexpr_from_str(&status_key, "sanity/01/status");
    z_owned_closure_sample_t callback;
    z_closure_sample(&callback, receive, NULL, NULL);
    z_owned_subscriber_t subscriber;
    if (z_declare_subscriber(z_session_loan(&session), &subscriber,
                             z_view_keyexpr_loan(&command_key), z_closure_sample_move(&callback),
                             NULL) < 0)
        return 4;
    z_owned_publisher_t publisher;
    z_publisher_options_t options;
    z_publisher_options_default(&options);
    options.is_express = true;
    options.congestion_control = Z_CONGESTION_CONTROL_DROP;
    options.priority = Z_PRIORITY_REAL_TIME;
    if (z_declare_publisher(z_session_loan(&session), &publisher, z_view_keyexpr_loan(&status_key),
                            &options) < 0)
        return 5;
    uint32_t start = millis(), step = 0;
    uint16_t previous[4] = {0};
    bool old_enabled = false, old_failsafe = false;
    uint8_t old_mode = 0;
    while (millis() - start < duration) {
        uint32_t now = millis();
        uint16_t values[4];
        uint8_t status[SZ_STATUS_SIZE], encoded[25];
        pthread_mutex_lock(&mutex);
        now = millis();
        bool failsafe = sz_output(&state, now, values), enabled = state.enabled;
        uint8_t active_mode = state.mode;
        sz_status(&state, now, unified || mode == SZ_DIRECT, unified || mode == SZ_DIRECT ? 7800 : 0,
                  unified || mode == SZ_DIRECT ? 0 : 65535, status);
        pthread_mutex_unlock(&mutex);
        if (memcmp(previous, values, sizeof(values)) || old_enabled != enabled ||
            old_failsafe != failsafe || old_mode != active_mode) {
            size_t n;
            if (unified && active_mode == SZ_CTBR) {
                n = sz_rc_msp(values, enabled, encoded);
            } else if (mode == SZ_CTBR) {
                sz_sbus(values, enabled, failsafe, encoded);
                n = 25;
            } else
                n = sz_motors(values, encoded);
            printf("{\"mode\":%u,\"enabled\":%s,\"failsafe\":%s,\"values\":[%u,%u,%u,%u],\"serial_hex\":\"",
                   active_mode, enabled ? "true" : "false", failsafe ? "true" : "false", values[0], values[1],
                   values[2], values[3]);
            for (size_t i = 0; i < n; ++i)
                printf("%02x", encoded[i]);
            printf("\"}\n");
            fflush(stdout);
            memcpy(previous, values, sizeof(values));
            old_enabled = enabled;
            old_failsafe = failsafe;
            old_mode = active_mode;
        }
        if (step++ % 2 == 0) {
            z_owned_bytes_t payload;
            z_bytes_copy_from_buf(&payload, status, sizeof(status));
            z_publisher_put(z_publisher_loan(&publisher), z_bytes_move(&payload), NULL);
        }
        struct timespec pause = {.tv_nsec = 10000000};
        nanosleep(&pause, NULL);
    }
    z_publisher_drop(z_publisher_move(&publisher));
    z_subscriber_drop(z_subscriber_move(&subscriber));
    z_session_drop(z_session_move(&session));
    return 0;
}
