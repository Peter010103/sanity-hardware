#include "sdkconfig.h"
#include "hx711.h"
#include "bench.h"
#include "telemetry_profile.h"
#include "fc_telemetry.h"
#include "test_config.h"
#include "session_watch.h"
#ifdef CONFIG_SANITY_MOCAP_STREAM
#include "mocap_state.h"
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
#include "estimator/runtime.h"
#include "mocap_clock_udp.h"
#endif
#endif
#ifdef CONFIG_SANITY_MSP_TUNING
#include "msp_tune.h"
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
#include "diagnostics.h"
#endif
#include "control.h"
#include "passthrough.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs.h"
#include "esp_wifi.h"
#ifdef CONFIG_IDF_TARGET_ESP32P4
#include "esp_hosted.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "zenoh-pico.h"
#include <stdio.h>
#include <string.h>
#ifdef CONFIG_SANITY_RAW_UDP
#include <fcntl.h>
#include <errno.h>
#include "lwip/sockets.h"
#include "lwip/inet.h"
#endif

#ifdef CONFIG_SANITY_DIRECT_BENCH
#define OUTPUT_MODE SZ_DIRECT
#else
#define OUTPUT_MODE SZ_CTBR
#endif
#if defined(CONFIG_SANITY_DIRECT_BENCH) || defined(CONFIG_SANITY_MSP_UNIFIED)
#define MSP_OUTPUT 1
#else
#define MSP_OUTPUT 0
#endif
#define FC_UART UART_NUM_2
/* Pico1.10.1 runtime constructors have large nested stack frames during z_open. */
#define ZENOH_OWNER_STACK 12288
static struct sz_state state;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
#ifdef CONFIG_SANITY_MOCAP_STREAM
static portMUX_TYPE mocap_lock = portMUX_INITIALIZER_UNLOCKED;
static struct sz_mocap_cache mocap_cache;
static struct {uint64_t nonce,host_us,received_us;uint32_t version;} mocap_clock;
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
static uint64_t mocap_current_epoch(void) {
    portENTER_CRITICAL(&mocap_lock);
    uint64_t epoch = mocap_cache.epoch;
    portEXIT_CRITICAL(&mocap_lock);
    return epoch;
}
#endif
#endif
static EventGroupHandle_t wifi_events;
static bool fc_ready = false, fc_ctbr_ready = false, fc_direct_ready = false, fc_disarmed = false;
static uint16_t voltage_mv = 0, fc_age = 65535;
static QueueHandle_t telemetry_events;
/* Authority/health must never wait behind the bulk diagnostic FIFO. */
static QueueHandle_t controller_telemetry_event;
static uint32_t telemetry_dropped;
static bool telemetry_enabled=true;
#ifdef CONFIG_SANITY_COMMS_TEST_ONLY
static uint8_t test_reply[SZ_TEST_CONFIG_SIZE];
static uint32_t test_reply_version;
static bool test_reply_pending;
#endif
static uint64_t command_rx_us;
#ifdef CONFIG_SANITY_PAPER_NATIVE
static struct sz_paper_link paper_link;
#endif
#ifdef CONFIG_SANITY_MSP_TUNING
static struct sz_tune_rpc tuning;
static uint8_t fc_profile;
#endif
static struct sz_fc_health esp_health;
static struct sz_telemetry_profile telemetry_profile;
static void telemetry_emit(struct sz_fc_record *record) {
    if(!sz_telemetry_allow(&telemetry_profile,record->type,record->command,esp_timer_get_time()))return;
    if (!__atomic_load_n(&telemetry_enabled,__ATOMIC_RELAXED)) return;
    if (sz_fc_is_controller_record(record)) {
        xQueueOverwrite(controller_telemetry_event,record);
        return;
    }
    if (xQueueSend(telemetry_events, record, 0) != pdTRUE)
        __atomic_add_fetch(&telemetry_dropped, 1, __ATOMIC_RELAXED);
}
static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static void stop(uint8_t reason) {
    (void)reason;
    portENTER_CRITICAL(&lock);
    sz_stop(&state);
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
    struct sz_state copy=state;
#endif
    portEXIT_CRITICAL(&lock);
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
    sz_diag_stop(&copy,reason,0,voltage_mv,fc_age);
#endif
}
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event=data;
        portENTER_CRITICAL(&lock);
        ++esp_health.disconnects;esp_health.disconnect_reason=event->reason;esp_health.last_disconnect_us=esp_timer_get_time();
        portEXIT_CRITICAL(&lock);
        xEventGroupClearBits(wifi_events, 1);
        stop(4);
        esp_wifi_connect();
    }
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        portENTER_CRITICAL(&lock);++esp_health.connects;portEXIT_CRITICAL(&lock);
        xEventGroupSetBits(wifi_events, 1);
    }
}
static void wifi_start(void) {
    wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#ifdef CONFIG_IDF_TARGET_ESP32P4
    /* Start the C6 only in receiver mode; USB passthrough must stay offline. */
    ESP_ERROR_CHECK(esp_hosted_init());
    ESP_ERROR_CHECK(esp_hosted_connect_to_slave());
#endif
    esp_netif_t *station = esp_netif_create_default_wifi_sta();
    if (!station)
        abort();
#ifdef CONFIG_SANITY_STATIC_IP
    esp_err_t dhcp = esp_netif_dhcpc_stop(station);
    if (dhcp != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)
        ESP_ERROR_CHECK(dhcp);
    esp_netif_ip_info_t ip = {0};
    ESP_ERROR_CHECK(esp_netif_str_to_ip4(CONFIG_SANITY_IPV4_ADDRESS, &ip.ip));
    ESP_ERROR_CHECK(esp_netif_str_to_ip4(CONFIG_SANITY_IPV4_NETMASK, &ip.netmask));
    ESP_ERROR_CHECK(esp_netif_str_to_ip4(CONFIG_SANITY_IPV4_GATEWAY, &ip.gw));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(station, &ip));
#endif
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
    wifi_config_t conf = {
        .sta = {.ssid = CONFIG_SANITY_WIFI_SSID, .password = CONFIG_SANITY_WIFI_PASSWORD}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &conf));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}
static void uart_start(void) {
    uart_config_t cfg = {.baud_rate =
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
                         921600,
#else
                         MSP_OUTPUT ? CONFIG_SANITY_MSP_BAUD : 100000,
#endif
                         .data_bits = UART_DATA_8_BITS,
                         .parity = MSP_OUTPUT ? UART_PARITY_DISABLE : UART_PARITY_EVEN,
                         .stop_bits = MSP_OUTPUT ? UART_STOP_BITS_1 : UART_STOP_BITS_2,
                         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(FC_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(FC_UART, CONFIG_SANITY_UART_TX, CONFIG_SANITY_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(FC_UART, 2048, 0, 0, NULL, 0));
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
    // Short clock replies must not wait behind a mostly empty RX FIFO.
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(FC_UART, 32));
    ESP_ERROR_CHECK(uart_set_rx_timeout(FC_UART, 2));
#endif
#ifdef CONFIG_SANITY_SBUS_INVERT
    if (!MSP_OUTPUT)
        ESP_ERROR_CHECK(uart_set_line_inverse(FC_UART, UART_SIGNAL_TXD_INV));
#endif
}
static void output_task(void *unused) {
    (void)unused;
    struct bench_state bench;
    bench_init(&bench);
    struct sz_fc_parser telemetry;
    sz_fc_init(&telemetry);
    uint32_t step = 0, enabled_since = 0;
    bool was_enabled = false;
    static const uint8_t checks[] = {1, 2, 3, 36, 90, 131, 119};
#ifndef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
    TickType_t wake = xTaskGetTickCount();
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
    uint64_t previous_poll=0;
#endif
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
    uint64_t next_command=0;
#endif
    for (;;) {
        uint32_t now = now_ms();
        uint8_t out[25], rx[512];
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
        uint64_t sensor_now=esp_timer_get_time();
        // Probe the clock 6--8 ms after the preceding 100 Hz command burst.
        bool clock_slot=!next_command || (sensor_now+4000>=next_command && sensor_now+2000<next_command);
        uint8_t sensor_request[64];size_t sensor_n=se_runtime_poll_in_slot(sensor_now,sensor_request,clock_slot);
        if(sensor_n)uart_write_bytes(FC_UART,sensor_request,sensor_n);
#endif
        uint16_t v[4];
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        uint8_t reason=0;
#endif
        if (MSP_OUTPUT) {
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            uint64_t poll=esp_timer_get_time();
            if(previous_poll && poll>previous_poll+12000) sz_diag_poll_gap(previous_poll,poll);
#endif
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
            // Wake on incoming data, then drain available bytes. Waiting for
            // a whole buffer or a periodic delay inflates clock-reply RTT.
            int n = uart_read_bytes(FC_UART, rx, 1, pdMS_TO_TICKS(1));
            if(n>0)n+=uart_read_bytes(FC_UART,rx+n,sizeof(rx)-n,0);
#else
            int n = uart_read_bytes(FC_UART, rx, sizeof(rx), 0);
#endif
            for (int i = 0; i < n; ++i) {
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
                se_runtime_byte(rx[i],esp_timer_get_time());
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
                if (bench.parse_state==6 && rx[i]==bench.checksum && bench.length==0 &&
                    (bench.command==200 || bench.command==214))
                    sz_diag_ack(bench.command,previous_poll,esp_timer_get_time());
#endif
                struct sz_fc_record sample;
                if (sz_fc_byte(&telemetry,rx[i],esp_timer_get_time(),&sample)) {
                    portENTER_CRITICAL(&lock);
                    sample.epoch=state.epoch;
                    portEXIT_CRITICAL(&lock);
#ifdef CONFIG_SANITY_MSP_TUNING
                    portENTER_CRITICAL(&lock);sz_tune_response(&tuning,&sample,esp_timer_get_time());portEXIT_CRITICAL(&lock);
#endif
#ifdef CONFIG_SANITY_PAPER_NATIVE
                    if(sample.command==255 && sample.length>=6 && sample.payload[1]==0x16 && sample.payload[2]==0x7f) {
                        portENTER_CRITICAL(&lock);
                        sz_paper_observe(&paper_link,state.epoch,now_ms(),sample.payload,
                            sample.type==SZ_FC_RESPONSE?sample.length:0);
                        portEXIT_CRITICAL(&lock);
                    }
#endif
                    telemetry_emit(&sample);
                }
                bench_byte(&bench, rx[i], now);
            }
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            previous_poll=poll;

#endif
        }
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
        uint64_t command_now=esp_timer_get_time();
        if(command_now<next_command)continue; // UART read above blocks/yields until data or one tick.
        next_command=command_now+10000;
#endif
        portENTER_CRITICAL(&lock);
        /* Read the clock after the lock: last_rx_ms may have advanced while
         * UART parsing or a competing callback ran before we acquired it. */
        now = now_ms();
        if (MSP_OUTPUT) {
            fc_direct_ready = bench_ready(&bench, now);
            fc_ctbr_ready = bench_ctbr_ready(&bench, now);
            fc_disarmed = bench_disarmed(&bench, now);
#ifdef CONFIG_SANITY_MSP_TUNING
            fc_profile=bench.pid_profile;
#endif
            fc_ready = state.mode == SZ_DIRECT ? fc_direct_ready : fc_ctbr_ready;
            voltage_mv =
                (bench.seen & (1u << 8)) && now - bench.stamp[8] < 150 ? bench.voltage_mv : 0;
            uint32_t age = (bench.seen & (1u << 7)) ? now - bench.stamp[7] : 65535;
            fc_age = age > 65535 ? 65535 : (uint16_t)age;
            if (!fc_ready && state.enabled) {
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
                reason=2;
#endif
                sz_stop(&state);
            }
            if (state.mode == SZ_DIRECT && state.enabled && !was_enabled)
                enabled_since = now;
            if (state.mode == SZ_DIRECT && state.enabled && now - enabled_since >= CONFIG_SANITY_BENCH_LIMIT_S * 1000u) {
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
                reason=3;
#endif
                sz_stop(&state);
            }
        }
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        bool before_output=state.enabled;
#endif
        bool failsafe = sz_output(&state, now, v), enabled = state.enabled;
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        if (before_output && !enabled) reason=1;
        uint64_t snapshot=esp_timer_get_time();
        struct sz_state copy=state;
#endif
        uint8_t mode = state.mode;
#ifdef CONFIG_SANITY_MSP_UNIFIED
        bool indi_requested = enabled && (mode==SZ_CTBR || mode==SZ_PAPER_NATIVE) && state.command.controller_selection && state.command.indi_requested;
#ifdef CONFIG_SANITY_PAPER_NATIVE
        struct sz_command paper_command=state.command;
        bool paper_available=sz_paper_forward(&paper_link,state.epoch,enabled);
        if(mode==SZ_PAPER_NATIVE && !paper_available && enabled) {sz_stop(&state);enabled=false;indi_requested=false;}
#endif
#endif
        struct sz_fc_record output={.type=SZ_FC_OUTPUT,.flags=1,.epoch=state.epoch,
            .session=state.valid?state.command.session:0,.sequence=state.valid?state.command.sequence:0,
            .command_rx_us=command_rx_us};
        was_enabled = enabled;
        portEXIT_CRITICAL(&lock);
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        if (reason) sz_diag_stop(&copy,reason,bench.seen,voltage_mv,fc_age);
#endif
        if (!MSP_OUTPUT) {
            sz_sbus(v, enabled, failsafe, out);
            uart_write_bytes(FC_UART, out, 25);
        } else {
#ifdef CONFIG_SANITY_MSP_UNIFIED
            uint16_t stopped[4] = {1000, 1000, 1000, 1000};
            /* Keep the FC disarmed motor cache STOP in CTBR, and ARM low in direct. */
            size_t rc_n = sz_rc_msp_select(v, (mode == SZ_CTBR || mode == SZ_PAPER_NATIVE) && enabled, indi_requested, out);
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            uint64_t rc_before=esp_timer_get_time();
#endif
            output.command=200;output.length=(uint8_t)(rc_n-6);memcpy(output.payload,out+5,output.length);
            output.start_us=esp_timer_get_time();
            int rc_written=uart_write_bytes(FC_UART, out, rc_n);
            (void)rc_written;
            output.end_us=esp_timer_get_time();telemetry_emit(&output);
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
            if(mode==SZ_CTBR) {
                struct sz_trace timing=sz_command_uart_trace(&copy,step,200,output.command_rx_us,
                    snapshot,output.start_us,output.end_us,rc_n,rc_written);
                sz_diag_command_timing(&timing);
            }
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            sz_diag_write(&copy,200,step,snapshot,rc_before,esp_timer_get_time(),rc_n);
#endif
#ifdef CONFIG_SANITY_PAPER_NATIVE
            if(mode==SZ_PAPER_NATIVE && paper_available) {
                /* RC companion/arming owner is enqueued first. Never send an
                 * enabled native packet after a short/failed fallback write. */
                if(!enabled) {paper_command.enabled=false;paper_command.indi_requested=false;}
                if(!enabled || rc_written==(int)rc_n) {
                    uint8_t native_frame[76];size_t native_n=sz_paper_after_rc(&paper_command,now_ms(),rc_n,rc_written,native_frame);
                    if(native_n) {
                        int written=uart_write_bytes(FC_UART,native_frame,native_n);
                        if(written!=(int)native_n) {portENTER_CRITICAL(&lock);sz_stop(&state);portEXIT_CRITICAL(&lock);}
                    }
                } else {portENTER_CRITICAL(&lock);sz_stop(&state);portEXIT_CRITICAL(&lock);}
            }
#endif
            sz_motors(mode == SZ_DIRECT ? v : stopped, out);
#else
            (void)mode;
            sz_motors(v, out);
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            uint64_t motor_before=esp_timer_get_time();
#endif
            output.command=214;output.length=8;memcpy(output.payload,out+5,8);
            output.start_us=esp_timer_get_time();
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
            int motor_written=
#endif
            uart_write_bytes(FC_UART, out, 14);
            output.end_us=esp_timer_get_time();telemetry_emit(&output);
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
            if(mode==SZ_DIRECT) {
                struct sz_trace timing=sz_command_uart_trace(&copy,step,214,output.command_rx_us,
                    snapshot,output.start_us,output.end_us,14,motor_written);
                sz_diag_command_timing(&timing);
            }
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            sz_diag_write(&copy,214,step,snapshot,motor_before,esp_timer_get_time(),14);
#endif
            bool tuning_sent=false;
#ifdef CONFIG_SANITY_MSP_TUNING
            uint8_t tuning_frame[SZ_TUNE_MAX+6];
            portENTER_CRITICAL(&lock);
            struct sz_tune_context context={.state=&state,.now_ms=now_ms(),.profile=fc_profile,.fc_ready=fc_ctbr_ready,.fc_disarmed=fc_disarmed};
            size_t tuning_n=sz_tune_poll(&tuning,&context,esp_timer_get_time(),tuning_frame);
            portEXIT_CRITICAL(&lock);
            if(tuning_n){uart_write_bytes(FC_UART,tuning_frame,tuning_n);tuning_sent=true;}
#endif
            if (!tuning_sent && step % 2 == 0) {
                size_t n = sz_msp(101, NULL, 0, out);
                uart_write_bytes(FC_UART, out, n);
            }
            if (!tuning_sent && step % 10 == 1) {
                size_t n = sz_msp(130, NULL, 0, out);
                uart_write_bytes(FC_UART, out, n);
            }
            if (!tuning_sent && step % 5 == 2) {
                size_t n = sz_msp(checks[(step / 5) % 7], NULL, 0, out);
                uart_write_bytes(FC_UART, out, n);
            }
#ifdef CONFIG_SANITY_PAPER_NATIVE
            if(!tuning_sent && step%10==6) {
                struct sz_fc_record timeout;
                if(sz_fc_query(&telemetry,SZ_FC_PAPER_QUERY_SLOT,esp_timer_get_time(),&timeout)) {
                    uint8_t query[12];size_t n=sz_paper_query(query);uart_write_bytes(FC_UART,query,n);
                } else if(timeout.type) {timeout.epoch=output.epoch;telemetry_emit(&timeout);}
            }
#endif
            size_t buffered=0;
            uart_get_buffered_data_len(FC_UART,&buffered);
            if (!tuning_sent && (bench.seen&7)==7 && buffered<1024) {
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
                /* Separate pending/backoff state: unsupported optional timing
                 * replies cannot stall the20Hz authority snapshot query. */
                if(step%5==3 && !telemetry.outstanding[4]) {
                    struct sz_fc_record timeout;
                    if(sz_fc_query(&telemetry,SZ_FC_TIMING_QUERY_SLOT,esp_timer_get_time(),&timeout)) {
                        uint8_t request[6];sz_fc_timing_query_payload(request);
                        size_t n=sz_msp(255,request,sizeof(request),out);uart_write_bytes(FC_UART,out,n);
                    } else if(timeout.type) {timeout.epoch=output.epoch;telemetry_emit(&timeout);}
                }
#endif
                if(step%5==4) {
                    struct sz_fc_record timeout;
                    if(sz_fc_query(&telemetry,255,esp_timer_get_time(),&timeout)) {
                        uint8_t request[6];sz_fc_controller_query_payload(request);
                        size_t n=sz_msp(255,request,sizeof(request),out);uart_write_bytes(FC_UART,out,n);
                    } else if(timeout.type) {timeout.epoch=output.epoch;telemetry_emit(&timeout);}
                }
                uint8_t queries[2]={102,step%2==0?104:step%10==1?139:step%10==5?105:0};
                for (unsigned i=0;i<2;++i) if (queries[i]) {
                    struct sz_fc_record timeout;
                    if (sz_fc_query(&telemetry,queries[i],esp_timer_get_time(),&timeout)) {
                        size_t n=sz_msp(queries[i],NULL,0,out);
                        uart_write_bytes(FC_UART,out,n);
                    } else if (timeout.type) {
                        timeout.epoch=output.epoch;telemetry_emit(&timeout);
                    }
                }
            }
        }
        ++step;
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
        taskYIELD(); // The next UART read supplies the bounded wait.
#else
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
#endif
    }
}
static void command_bytes(const uint8_t *packet,size_t size,uint64_t received_us) {
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
    uint64_t entered=received_us;
#endif
    struct sz_command command={0};bool ok=size==SZ_COMMAND_SIZE || size==SZ_PAPER_COMMAND_SIZE;
    ok = ok && sz_decode(packet, size, &command);
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
    const bool decoded=ok;
    uint8_t stage_reason=ok?SZ_CMD_ACCEPTED:SZ_CMD_DECODE;
#endif
    ok = ok && sz_profile_allows(&command);
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
    if(decoded && !ok)stage_reason=SZ_CMD_PROFILE;
#endif
    portENTER_CRITICAL(&lock);
    uint16_t measured_voltage=voltage_mv;
    portEXIT_CRITICAL(&lock);
    ok = ok && sz_calibrate(&command,measured_voltage);
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
    if(!ok && !stage_reason)stage_reason=SZ_CMD_CALIBRATION;
#endif
    portENTER_CRITICAL(&lock);
    uint32_t now = now_ms();
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
    struct sz_state before=state;
    uint64_t previous_rx=(state.valid && state.epoch==command.epoch &&
        state.command.session==command.session)?command_rx_us:0;
#endif
    bool accepted=false;
    bool ready = command.mode == SZ_DIRECT ? fc_direct_ready : fc_ctbr_ready;
#ifdef CONFIG_SANITY_PAPER_NATIVE
    if(command.paper_native && command.enabled) ready=ready && sz_paper_admit(&paper_link,state.epoch,now);
#endif
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
    if(ok && MSP_OUTPUT && command.enabled && !ready)stage_reason=SZ_CMD_FC_NOT_READY;
#endif
    if (ok && !(MSP_OUTPUT && command.enabled && !ready)) {
#ifdef CONFIG_SANITY_MSP_UNIFIED
        accepted=sz_accept_msp(&state, &command, now, fc_disarmed);
        fc_ready = state.mode == SZ_DIRECT ? fc_direct_ready : fc_ctbr_ready;
#else
        accepted=sz_accept(&state, &command, now);
#endif
    } else
        ++state.rejected;
    if (accepted) {
        command_rx_us=received_us;
#ifdef CONFIG_SANITY_PAPER_NATIVE
        if(command.paper_native)paper_link.negotiated=command.enabled;
#endif
    }
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
    const uint64_t decision_done=esp_timer_get_time();
    const uint32_t rejected_count=state.rejected;
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
    uint64_t current_epoch=state.epoch;
#endif
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    uint32_t last_sequence=state.command.sequence;
    bool epoch_match=command.epoch==state.epoch,session_match=command.session==state.command.session;
#endif
    portEXIT_CRITICAL(&lock);
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
    struct sz_trace timing=sz_command_decision_trace(&command,decoded,accepted,
        sz_command_reject_reason(&before,&command,accepted,stage_reason,now),
        now,received_us,decision_done,previous_rx,rejected_count);
    sz_diag_command_timing(&timing);
#endif
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    static uint32_t last_rejection_log;
    if(ok && !accepted && now-last_rejection_log>=1000){
        last_rejection_log=now;
        ESP_LOGW("sanity_rx","reject now=%lu lease=%lu age=%lu ttl=%u seq=%lu last=%lu device=%u mode=%u enabled=%u epoch_match=%u session_match=%u",
            (unsigned long)now,(unsigned long)command.lease_ms,(unsigned long)(now-command.lease_ms),command.ttl_ms,
            (unsigned long)command.sequence,(unsigned long)last_sequence,command.device,command.mode,command.enabled,epoch_match,session_match);
    }
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
    sz_diag_epoch(current_epoch);
    if (ok) sz_diag_command(&command,accepted,entered,esp_timer_get_time(),now);
#endif
}
#ifndef CONFIG_SANITY_RAW_UDP
#ifdef CONFIG_SANITY_MOCAP_STREAM
static void mocap_received(z_loaned_sample_t *sample,void *unused) {
    (void)unused;uint64_t received=esp_timer_get_time();uint8_t bytes[SZ_MOCAP_SIZE];
    const z_loaned_bytes_t *payload=z_sample_payload(sample);
    struct sz_mocap_sample decoded;bool ok=false;
    if(z_bytes_len(payload)==sizeof(bytes)) {
        z_bytes_reader_t reader=z_bytes_get_reader(payload);
        ok=z_bytes_reader_read(&reader,bytes,sizeof(bytes))==sizeof(bytes) &&
           sz_mocap_decode(bytes,sizeof(bytes),CONFIG_SANITY_DEVICE_ID,&decoded);
    }
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
    if(ok)se_runtime_pose(&decoded,received);
#endif
    portENTER_CRITICAL(&mocap_lock);
    if(ok)sz_mocap_accept(&mocap_cache,&decoded,received);else mocap_cache.rejected++;
    portEXIT_CRITICAL(&mocap_lock);
}
static void mocap_clock_received(z_loaned_sample_t *sample,void *unused) {
    (void)unused;uint64_t received=esp_timer_get_time(),nonce,host;uint8_t bytes[SZ_MOCAP_CLOCK_REQUEST_SIZE];
    const z_loaned_bytes_t *payload=z_sample_payload(sample);
    if(z_bytes_len(payload)!=sizeof(bytes))return;
    z_bytes_reader_t reader=z_bytes_get_reader(payload);
    if(z_bytes_reader_read(&reader,bytes,sizeof(bytes))!=sizeof(bytes) ||
       !sz_mocap_clock_request(bytes,sizeof(bytes),CONFIG_SANITY_DEVICE_ID,&nonce,&host))return;
    portENTER_CRITICAL(&mocap_lock);
    mocap_clock.nonce=nonce;mocap_clock.host_us=host;mocap_clock.received_us=received;mocap_clock.version++;
    portEXIT_CRITICAL(&mocap_lock);
}
#endif
static void command_received(z_loaned_sample_t *sample, void *context) {
    (void)context;uint64_t received_us=esp_timer_get_time();uint8_t packet[SZ_PAPER_COMMAND_SIZE];
    const z_loaned_bytes_t *payload=z_sample_payload(sample);size_t length=z_bytes_len(payload);
    if(length!=SZ_COMMAND_SIZE && length!=SZ_PAPER_COMMAND_SIZE)return;
    z_bytes_reader_t reader=z_bytes_get_reader(payload);
    if(z_bytes_reader_read(&reader,packet,length)==length)command_bytes(packet,length,received_us);
}

#ifdef CONFIG_SANITY_MSP_TUNING
static void tuning_received(z_loaned_sample_t *sample,void *unused) {
    (void)unused;uint8_t bytes[SZ_TUNE_HEADER+SZ_TUNE_MAX];
    const z_loaned_bytes_t *payload=z_sample_payload(sample);size_t n=z_bytes_len(payload);
    if(n<32||n>sizeof(bytes))return;
    z_bytes_reader_t reader=z_bytes_get_reader(payload);
    if(z_bytes_reader_read(&reader,bytes,n)!=n)return;
    portENTER_CRITICAL(&lock);
    struct sz_tune_context context={.state=&state,.now_ms=now_ms(),.profile=fc_profile,.fc_ready=fc_ctbr_ready,.fc_disarmed=fc_disarmed};
    sz_tune_submit(&tuning,bytes,n,&context,esp_timer_get_time());portEXIT_CRITICAL(&lock);
}
#endif
#ifdef CONFIG_SANITY_COMMS_TEST_ONLY
static void telemetry_control(z_loaned_sample_t *sample, void *context) {
    (void)context;uint8_t packet[SZ_TEST_CONFIG_SIZE];
    const z_loaned_bytes_t *payload=z_sample_payload(sample);
    if (z_bytes_len(payload)!=sizeof(packet)) return;
    z_bytes_reader_t reader=z_bytes_get_reader(payload);
    if (z_bytes_reader_read(&reader,packet,sizeof(packet))!=sizeof(packet)) return;
    portENTER_CRITICAL(&lock);
    int result=sz_test_config(packet,sizeof(packet),CONFIG_SANITY_DEVICE_ID,
        state.command.session,state.valid,state.enabled,telemetry_enabled,test_reply);
    if (result>=0) {
        if(result) __atomic_store_n(&telemetry_enabled,packet[6]!=0,__ATOMIC_RELAXED);
        ++test_reply_version;test_reply_pending=true;
    }
    portEXIT_CRITICAL(&lock);
}
#endif
#endif /* Zenoh receive callbacks */
static void publish_telemetry_to(sz_diag_send_fn send,void *context) {
    static uint64_t next;
    if(CONFIG_SANITY_DIAGNOSTIC_LEVEL!=0){uint64_t now=esp_timer_get_time();if(now<next)return;next=now+50000;}
    uint8_t bytes[SZ_FC_BATCH_SIZE];size_t length=SZ_FC_HEADER_SIZE;uint16_t count=0;struct sz_fc_record sample;
    bool enabled=__atomic_load_n(&telemetry_enabled,__ATOMIC_RELAXED);
    /* Keep only the newest controller snapshot and serialize it first. */
    if (xQueueReceive(controller_telemetry_event,&sample,0)==pdTRUE && enabled) {
        size_t n=sz_fc_encode(&sample,bytes+length,sizeof(bytes)-length);
        if(n){length+=n;++count;}
    }
    /* Eight maximum-size records fit the1400-byte packet. Four per~60ms
     * could not drain the minimal profile once20Hz controller polling ran. */
    unsigned limit=CONFIG_SANITY_DIAGNOSTIC_LEVEL==0?31:8;
    while(count<limit && xQueuePeek(telemetry_events,&sample,0)==pdTRUE) {
        size_t n=sz_fc_encode(&sample,bytes+length,sizeof(bytes)-length);
        if(enabled && !n)break;
        xQueueReceive(telemetry_events,&sample,0);
        if(enabled){length+=n;++count;}
    }
    if(!count)return;
    sz_fc_batch_header(bytes,CONFIG_SANITY_DEVICE_ID,count,__atomic_load_n(&telemetry_dropped,__ATOMIC_RELAXED));
    uint64_t before=esp_timer_get_time();int result=send(bytes,length,context);
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
    sz_diag_publish_time(4,before,esp_timer_get_time(),result);
#else
    (void)before;
#endif
    if(result<0)__atomic_add_fetch(&telemetry_dropped,count,__ATOMIC_RELAXED);
}
#ifndef CONFIG_SANITY_RAW_UDP
static int telemetry_zenoh_send(const uint8_t *bytes,size_t length,void *context) {
    z_owned_bytes_t payload;z_bytes_copy_from_buf(&payload,bytes,length);
    return z_publisher_put((const z_loaned_publisher_t *)context,z_bytes_move(&payload),NULL);
}
static void publish_telemetry(const z_loaned_publisher_t *pub) {publish_telemetry_to(telemetry_zenoh_send,(void *)pub);}
#endif
static void publish_health(void) {
    static uint64_t next;
    uint64_t now=esp_timer_get_time();if(now<next)return;next=now+1000000;
    struct sz_fc_health health;
    portENTER_CRITICAL(&lock);health=esp_health;uint64_t epoch=state.epoch;portEXIT_CRITICAL(&lock);
    health.free_heap=esp_get_free_heap_size();health.min_free_heap=esp_get_minimum_free_heap_size();
    wifi_ap_record_t ap;if(esp_wifi_sta_get_ap_info(&ap)==ESP_OK){health.rssi=ap.rssi;health.channel=ap.primary;}
    struct sz_fc_record record;sz_fc_health_record(&record,&health,epoch,now);telemetry_emit(&record);
}

#ifndef CONFIG_SANITY_RAW_UDP
#ifdef CONFIG_SANITY_SPLIT_TELEMETRY
static void telemetry_task(void *unused) {
    (void)unused;
    for (;;) {
        xEventGroupWaitBits(wifi_events,1,pdFALSE,pdTRUE,portMAX_DELAY);
        z_owned_config_t config;z_config_default(&config);
        zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MODE_KEY,"client");
        zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_CONNECT_KEY,CONFIG_SANITY_TELEMETRY_ENDPOINT);
        zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MULTICAST_SCOUTING_KEY,"false");
        z_owned_session_t session;
        z_task_attr_t task={.name="sz_telem_rx",.priority=6,.stack_depth=6144};
        z_open_options_t open_opts;z_open_options_default(&open_opts);open_opts.executor_task_attributes=&task;
        if(z_open(&session,z_config_move(&config),&open_opts)<0) {vTaskDelay(pdMS_TO_TICKS(1000));continue;}
        z_owned_publisher_t pub;
        z_internal_null(&pub);
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        z_owned_publisher_t traces;
        z_internal_null(&traces);
#endif
        struct sz_session_watch connection;
        z_owned_transport_events_listener_t listener;
        bool watched=false;
        z_publisher_options_t opts;z_publisher_options_default(&opts);
        opts.congestion_control=Z_CONGESTION_CONTROL_DROP;opts.priority=Z_PRIORITY_BACKGROUND;opts.is_express=true;
#ifdef Z_FEATURE_UNSTABLE_API
        opts.reliability=Z_RELIABILITY_BEST_EFFORT;
#endif
        char name[80];snprintf(name,sizeof(name),"sanity/%02d/telemetry/fc",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_t key;z_view_keyexpr_from_str(&key,name);
        if(z_declare_publisher(z_session_loan(&session),&pub,z_view_keyexpr_loan(&key),&opts)<0) goto telemetry_cleanup;
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        snprintf(name,sizeof(name),"sanity/%02d/diagnostics/trace",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&key,name);
        if(z_declare_publisher(z_session_loan(&session),&traces,z_view_keyexpr_loan(&key),&opts)<0) goto telemetry_cleanup;
#endif
        watched=sz_session_watch_start(z_session_loan(&session),&connection,&listener)>=0;
        if(!watched) goto telemetry_cleanup;
        /* A lost transport does not set session_is_closed in pinned Pico1.10.1.
         * Follow peer connectivity; publication can otherwise return OK forever. */
        while((xEventGroupGetBits(wifi_events)&1) && sz_session_watch_connected(&connection) && !z_session_is_closed(z_session_loan(&session))) {
            publish_telemetry(z_publisher_loan(&pub));
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            sz_diag_flush_traces(z_publisher_loan(&traces));
#endif
            vTaskDelay(pdMS_TO_TICKS(20));
        }
telemetry_cleanup:
        /* Peer loss may destroy Pico's TX mutex without closing the session.
         * Fence network I/O and join callbacks BEFORE dropping any entities. */
        if(z_close(z_session_loan_mut(&session),NULL)<0) abort();
        if(watched) z_transport_events_listener_drop(z_transport_events_listener_move(&listener));
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        z_publisher_drop(z_publisher_move(&traces));
#endif
        z_publisher_drop(z_publisher_move(&pub));z_session_drop(z_session_move(&session));vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif
static void zenoh_task(void *unused) {
    (void)unused;
    char key[80];
    for (;;) {
        xEventGroupWaitBits(wifi_events, 1, pdFALSE, pdTRUE, portMAX_DELAY);
        uint64_t epoch = ((uint64_t)esp_random() << 32) | esp_random();
        if (!epoch)
            epoch = 1;
        portENTER_CRITICAL(&lock);
        sz_init(&state, CONFIG_SANITY_DEVICE_ID, OUTPUT_MODE, epoch);
        command_rx_us=0;
        portEXIT_CRITICAL(&lock);
#ifdef CONFIG_SANITY_MOCAP_STREAM
        portENTER_CRITICAL(&mocap_lock);
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
        se_runtime_epoch(epoch);
#endif
        sz_mocap_init(&mocap_cache,CONFIG_SANITY_DEVICE_ID,epoch);memset(&mocap_clock,0,sizeof(mocap_clock));
        portEXIT_CRITICAL(&mocap_lock);
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        sz_diag_epoch(epoch);
#endif
        z_owned_config_t config;
        z_config_default(&config);
        zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, "client");
        zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY,
                         CONFIG_SANITY_ZENOH_ENDPOINT);
        zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MULTICAST_SCOUTING_KEY, "false");
        z_owned_session_t session;
        z_open_options_t open_opts;z_open_options_default(&open_opts);
#if defined(CONFIG_SANITY_COMMS_TEST_ONLY) || defined(CONFIG_SANITY_SPLIT_TELEMETRY)
        z_task_attr_t task={.name="sz_control_rx",.priority=14,.stack_depth=6144};
        open_opts.executor_task_attributes=&task;
#endif
        if (z_open(&session, z_config_move(&config), &open_opts) < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        z_owned_subscriber_t subscriber;
        z_internal_null(&subscriber);
        z_owned_publisher_t publisher;
        z_internal_null(&publisher);
#ifdef CONFIG_SANITY_MOCAP_STREAM
        z_owned_subscriber_t mocap_sub,mocap_clock_sub;
        z_owned_publisher_t mocap_pub,mocap_clock_pub;
        z_internal_null(&mocap_sub);z_internal_null(&mocap_clock_sub);
        z_internal_null(&mocap_pub);z_internal_null(&mocap_clock_pub);
        uint64_t mocap_next_status=0;uint32_t mocap_clock_sent=0;
#endif
#ifdef CONFIG_SANITY_MSP_TUNING
        z_owned_subscriber_t tune_sub;z_owned_publisher_t tune_pub;z_owned_closure_sample_t tune_closure;
        z_internal_null(&tune_sub);z_internal_null(&tune_pub);
#endif
#ifdef CONFIG_SANITY_COMMS_TEST_ONLY
        z_owned_subscriber_t test_sub;z_owned_closure_sample_t test_closure;z_owned_publisher_t test_pub;
        z_internal_null(&test_sub);z_internal_null(&test_pub);
#endif
#ifndef CONFIG_SANITY_SPLIT_TELEMETRY
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
        z_owned_publisher_t estimator_pub;z_internal_null(&estimator_pub);
#endif
        z_owned_publisher_t telemetry_pub;
        z_internal_null(&telemetry_pub);
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        z_owned_publisher_t pong_pub;
        z_internal_null(&pong_pub);
#ifndef CONFIG_SANITY_SPLIT_TELEMETRY
        z_owned_publisher_t trace_pub;
        z_internal_null(&trace_pub);
#endif
        z_owned_subscriber_t probe_sub;
        z_internal_null(&probe_sub);
#endif
#if Z_FEATURE_CONNECTIVITY == 1
        struct sz_session_watch control_connection;
        z_owned_transport_events_listener_t control_listener;
        bool control_watched=false;
#endif
        z_owned_closure_sample_t closure;
        z_view_keyexpr_t command_key, status_key;
        snprintf(key, sizeof(key), "sanity/%02d/command/%s", CONFIG_SANITY_DEVICE_ID,
#ifdef CONFIG_SANITY_MSP_UNIFIED
                 "*"
#else
                 OUTPUT_MODE == SZ_CTBR ? "ctbr" : "direct_thrust"
#endif
        );
        z_view_keyexpr_from_str(&command_key, key);
        z_closure_sample(&closure, command_received, NULL, NULL);
        if (z_declare_subscriber(z_session_loan(&session), &subscriber,
                                 z_view_keyexpr_loan(&command_key), z_closure_sample_move(&closure),
                                 NULL) < 0) {
            goto control_cleanup;
        }
        char status_name[64];
        snprintf(status_name, sizeof(status_name), "sanity/%02d/status", CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&status_key, status_name);
        z_publisher_options_t opts;
        z_publisher_options_default(&opts);
#ifdef Z_FEATURE_UNSTABLE_API
        opts.reliability=Z_RELIABILITY_BEST_EFFORT;
#endif
        opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
        opts.priority = Z_PRIORITY_REAL_TIME;
        opts.is_express = true;
        if (z_declare_publisher(z_session_loan(&session), &publisher,
                                z_view_keyexpr_loan(&status_key), &opts) < 0) {
            goto control_cleanup;
        }
#ifdef CONFIG_SANITY_MSP_TUNING
        char tune_name[80];snprintf(tune_name,sizeof(tune_name),"sanity/%02d/tuning/msp/request",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_t tune_key;z_view_keyexpr_from_str(&tune_key,tune_name);z_closure_sample(&tune_closure,tuning_received,NULL,NULL);
        if(z_declare_subscriber(z_session_loan(&session),&tune_sub,z_view_keyexpr_loan(&tune_key),z_closure_sample_move(&tune_closure),NULL)<0) goto control_cleanup;
        snprintf(tune_name,sizeof(tune_name),"sanity/%02d/tuning/msp/reply",CONFIG_SANITY_DEVICE_ID);z_view_keyexpr_from_str(&tune_key,tune_name);
        if(z_declare_publisher(z_session_loan(&session),&tune_pub,z_view_keyexpr_loan(&tune_key),&opts)<0) goto control_cleanup;
        uint32_t tune_version=0;
#endif
#ifdef CONFIG_SANITY_MOCAP_STREAM
        z_view_keyexpr_t mocap_key;z_owned_closure_sample_t mocap_closure;
        snprintf(key,sizeof(key),"sanity/%02d/state/mocap",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&mocap_key,key);z_closure_sample(&mocap_closure,mocap_received,NULL,NULL);
        if(z_declare_subscriber(z_session_loan(&session),&mocap_sub,z_view_keyexpr_loan(&mocap_key),z_closure_sample_move(&mocap_closure),NULL)<0)goto control_cleanup;
        snprintf(key,sizeof(key),"sanity/%02d/state/mocap/clock/request",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&mocap_key,key);z_closure_sample(&mocap_closure,mocap_clock_received,NULL,NULL);
        if(z_declare_subscriber(z_session_loan(&session),&mocap_clock_sub,z_view_keyexpr_loan(&mocap_key),z_closure_sample_move(&mocap_closure),NULL)<0)goto control_cleanup;
        snprintf(key,sizeof(key),"sanity/%02d/state/mocap/status",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&mocap_key,key);
        if(z_declare_publisher(z_session_loan(&session),&mocap_pub,z_view_keyexpr_loan(&mocap_key),&opts)<0)goto control_cleanup;
        snprintf(key,sizeof(key),"sanity/%02d/state/mocap/clock/reply",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&mocap_key,key);
        if(z_declare_publisher(z_session_loan(&session),&mocap_clock_pub,z_view_keyexpr_loan(&mocap_key),&opts)<0)goto control_cleanup;
#endif
#ifdef CONFIG_SANITY_COMMS_TEST_ONLY
        char test_name[80];snprintf(test_name,sizeof(test_name),"sanity/%02d/diagnostics/telemetry/set",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_t test_key;z_view_keyexpr_from_str(&test_key,test_name);z_closure_sample(&test_closure,telemetry_control,NULL,NULL);
        if(z_declare_subscriber(z_session_loan(&session),&test_sub,z_view_keyexpr_loan(&test_key),z_closure_sample_move(&test_closure),NULL)<0) goto control_cleanup;
        snprintf(test_name,sizeof(test_name),"sanity/%02d/diagnostics/telemetry/ack",CONFIG_SANITY_DEVICE_ID);z_view_keyexpr_from_str(&test_key,test_name);
        if(z_declare_publisher(z_session_loan(&session),&test_pub,z_view_keyexpr_loan(&test_key),&opts)<0) goto control_cleanup;
#endif
#ifndef CONFIG_SANITY_SPLIT_TELEMETRY
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
        z_view_keyexpr_t estimator_key;char estimator_name[64];
        snprintf(estimator_name,sizeof(estimator_name),"sanity/%02d/observation/records",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&estimator_key,estimator_name);
        z_publisher_options_t estimator_options=opts;estimator_options.priority=Z_PRIORITY_BACKGROUND;
        if(z_declare_publisher(z_session_loan(&session),&estimator_pub,z_view_keyexpr_loan(&estimator_key),&estimator_options)<0)goto control_cleanup;
#endif
        z_view_keyexpr_t telemetry_key;
        char telemetry_name[64];
        snprintf(telemetry_name,sizeof(telemetry_name),"sanity/%02d/telemetry/fc",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&telemetry_key,telemetry_name);
        z_publisher_options_t telemetry_options;
        z_publisher_options_default(&telemetry_options);
        telemetry_options.congestion_control=Z_CONGESTION_CONTROL_DROP;
        telemetry_options.priority=Z_PRIORITY_BACKGROUND;
        telemetry_options.is_express=true;
#ifdef Z_FEATURE_UNSTABLE_API
        telemetry_options.reliability=Z_RELIABILITY_BEST_EFFORT;
#endif
        if (z_declare_publisher(z_session_loan(&session),&telemetry_pub,z_view_keyexpr_loan(&telemetry_key),&telemetry_options)<0) {
            goto control_cleanup;
        }
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        z_owned_closure_sample_t probe_closure;
        z_view_keyexpr_t diag_key;
        char diag_name[64];
        snprintf(diag_name,sizeof(diag_name),"sanity/%02d/diagnostics/pong",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&diag_key,diag_name);
        if (z_declare_publisher(z_session_loan(&session),&pong_pub,z_view_keyexpr_loan(&diag_key),&opts)<0) goto control_cleanup;
#ifndef CONFIG_SANITY_SPLIT_TELEMETRY
        snprintf(diag_name,sizeof(diag_name),"sanity/%02d/diagnostics/trace",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&diag_key,diag_name);
        if (z_declare_publisher(z_session_loan(&session),&trace_pub,z_view_keyexpr_loan(&diag_key),&opts)<0) goto control_cleanup;
#endif
        snprintf(diag_name,sizeof(diag_name),"sanity/%02d/diagnostics/ping",CONFIG_SANITY_DEVICE_ID);
        z_view_keyexpr_from_str(&diag_key,diag_name);
        z_closure_sample(&probe_closure,sz_diag_probe,NULL,NULL);
        if (z_declare_subscriber(z_session_loan(&session),&probe_sub,z_view_keyexpr_loan(&diag_key),z_closure_sample_move(&probe_closure),NULL)<0) goto control_cleanup;
#endif
#if Z_FEATURE_CONNECTIVITY == 1
        control_watched=sz_session_watch_start(z_session_loan(&session),&control_connection,&control_listener)>=0;
        if(!control_watched) ESP_LOGE("sanity","Cannot watch control peer; recreating session");
#endif
        ESP_LOGI("sanity", "Zenoh connected; mode=%s, drone=%d",

#ifdef CONFIG_SANITY_MSP_UNIFIED
                 "MSP UNIFIED",
#else
                 OUTPUT_MODE == SZ_CTBR ? "CTBR SBUS" : "MSP BENCH",
#endif
                 CONFIG_SANITY_DEVICE_ID);

        while ((xEventGroupGetBits(wifi_events) & 1) &&
#if Z_FEATURE_CONNECTIVITY == 1
               control_watched && sz_session_watch_connected(&control_connection) &&
#endif
               !z_session_is_closed(z_session_loan(&session))) {
            uint8_t bytes[SZ_STATUS_SIZE];
            portENTER_CRITICAL(&lock);
            uint32_t now = now_ms();
            sz_status(&state, now, fc_ready, voltage_mv, fc_age, bytes);
            portEXIT_CRITICAL(&lock);
            z_owned_bytes_t payload;
            z_bytes_copy_from_buf(&payload, bytes, sizeof(bytes));
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            uint64_t status_put_start=esp_timer_get_time();
#endif
            int put_result=z_publisher_put(z_publisher_loan(&publisher), z_bytes_move(&payload), NULL);
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            sz_diag_publish_time(1,status_put_start,esp_timer_get_time(),put_result);
#endif
            /* DROP/congestion is not peer loss. Recreating a live session here
             * replaces the receiver epoch and invalidates command ownership.
             * The loop's peer/Wi-Fi/closed checks own reconnect decisions. */
            (void)put_result;
#ifdef CONFIG_SANITY_MOCAP_STREAM
            /* Copy under a separate short lock; encode/publish outside it. */
            uint64_t mc_nonce,mc_host,mc_received;uint32_t mc_version;struct sz_mocap_cache mc;
            portENTER_CRITICAL(&mocap_lock);
            mc=mocap_cache;mc_nonce=mocap_clock.nonce;mc_host=mocap_clock.host_us;
            mc_received=mocap_clock.received_us;mc_version=mocap_clock.version;
            portEXIT_CRITICAL(&mocap_lock);
            if(mc_nonce && mc_version!=mocap_clock_sent) {
                uint8_t reply[SZ_MOCAP_CLOCK_REPLY_SIZE];
                sz_mocap_clock_reply(CONFIG_SANITY_DEVICE_ID,mc.epoch,mc_nonce,mc_host,mc_received,esp_timer_get_time(),reply);
                z_owned_bytes_t response;z_bytes_copy_from_buf(&response,reply,sizeof(reply));
                if(z_publisher_put(z_publisher_loan(&mocap_clock_pub),z_bytes_move(&response),NULL)>=0)mocap_clock_sent=mc_version;
            }
            uint64_t mc_now=esp_timer_get_time();
            if(mc_now>=mocap_next_status) {
                uint8_t status[SZ_MOCAP_STATUS_SIZE];sz_mocap_status(&mc,mc_now,status);
                z_owned_bytes_t response;z_bytes_copy_from_buf(&response,status,sizeof(status));
                z_publisher_put(z_publisher_loan(&mocap_pub),z_bytes_move(&response),NULL);mocap_next_status=mc_now+100000;
            }
#endif
#ifdef CONFIG_SANITY_MSP_TUNING
            uint8_t tuning_reply[SZ_TUNE_REPLY_MAX];size_t tuning_size;uint32_t version;
            portENTER_CRITICAL(&lock);version=tuning.reply_version;tuning_size=tuning.reply_length;memcpy(tuning_reply,tuning.reply,tuning_size);portEXIT_CRITICAL(&lock);
            if(tuning_size && version!=tune_version){z_owned_bytes_t response;z_bytes_copy_from_buf(&response,tuning_reply,tuning_size);if(z_publisher_put(z_publisher_loan(&tune_pub),z_bytes_move(&response),NULL)>=0)tune_version=version;}
#endif
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
            // Bundle unchanged, independently checksummed SE01 records.
            // Bound each network cycle to two messages of at most 16 records.
            static uint8_t observation_batch[4096];
            for(unsigned batch=0;batch<2;batch++){
                size_t length=0;
                for(unsigned count=0;count<16;count++){
                    size_t rn=se_runtime_record(observation_batch+length);if(!rn)break;
                    length+=rn;
                }
                if(!length)break;
                z_owned_bytes_t observation;z_bytes_copy_from_buf(&observation,observation_batch,length);
                z_publisher_put(z_publisher_loan(&estimator_pub),z_bytes_move(&observation),NULL);
            }
#endif
            publish_health();
#ifndef CONFIG_SANITY_SPLIT_TELEMETRY
            publish_telemetry(z_publisher_loan(&telemetry_pub));
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            sz_diag_flush_pongs(z_publisher_loan(&pong_pub));
#ifndef CONFIG_SANITY_SPLIT_TELEMETRY
            sz_diag_flush_traces(z_publisher_loan(&trace_pub));
#endif
#endif
#ifdef CONFIG_SANITY_COMMS_TEST_ONLY
            uint8_t reply[SZ_TEST_CONFIG_SIZE];uint32_t version;
            portENTER_CRITICAL(&lock);bool pending=test_reply_pending;version=test_reply_version;memcpy(reply,test_reply,sizeof(reply));portEXIT_CRITICAL(&lock);
            if(pending) {
                z_owned_bytes_t response;z_bytes_copy_from_buf(&response,reply,sizeof(reply));
                if(z_publisher_put(z_publisher_loan(&test_pub),z_bytes_move(&response),NULL)>=0) {
                    portENTER_CRITICAL(&lock);if(test_reply_version==version)test_reply_pending=false;portEXIT_CRITICAL(&lock);
                }
            }
#endif
            vTaskDelay(pdMS_TO_TICKS(20));
        }
control_cleanup:
        stop(5);
#ifdef CONFIG_SANITY_MOCAP_STREAM
        portENTER_CRITICAL(&mocap_lock);sz_mocap_init(&mocap_cache,CONFIG_SANITY_DEVICE_ID,0);portEXIT_CRITICAL(&mocap_lock);
#endif
        /* STOP precedes potentially blocking shutdown. Closing first prevents
         * undeclare/filter cleanup from transmitting through a cleared transport. */
        if(z_close(z_session_loan_mut(&session),NULL)<0) abort();
#ifdef CONFIG_SANITY_MOCAP_STREAM
        z_subscriber_drop(z_subscriber_move(&mocap_sub));z_subscriber_drop(z_subscriber_move(&mocap_clock_sub));
        z_publisher_drop(z_publisher_move(&mocap_pub));z_publisher_drop(z_publisher_move(&mocap_clock_pub));
#endif
#if Z_FEATURE_CONNECTIVITY == 1
        if(control_watched) z_transport_events_listener_drop(z_transport_events_listener_move(&control_listener));
#endif
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        z_subscriber_drop(z_subscriber_move(&probe_sub));
        z_publisher_drop(z_publisher_move(&pong_pub));
#ifndef CONFIG_SANITY_SPLIT_TELEMETRY
        z_publisher_drop(z_publisher_move(&trace_pub));
#endif
#endif
#ifndef CONFIG_SANITY_SPLIT_TELEMETRY
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
        z_publisher_drop(z_publisher_move(&estimator_pub));
#endif
        z_publisher_drop(z_publisher_move(&telemetry_pub));
#endif
#ifdef CONFIG_SANITY_COMMS_TEST_ONLY
        z_subscriber_drop(z_subscriber_move(&test_sub));z_publisher_drop(z_publisher_move(&test_pub));
#endif
#ifdef CONFIG_SANITY_MSP_TUNING
        z_subscriber_drop(z_subscriber_move(&tune_sub));z_publisher_drop(z_publisher_move(&tune_pub));
#endif
        z_publisher_drop(z_publisher_move(&publisher));
        z_subscriber_drop(z_subscriber_move(&subscriber));
        z_session_drop(z_session_move(&session));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif /* Zenoh tasks */
#ifdef CONFIG_SANITY_RAW_UDP
static int raw_socket=-1;
static struct sockaddr_in raw_host;
static int raw_send(const uint8_t *data,size_t size,void *unused) {
    (void)unused;
    return sendto(raw_socket,data,size,MSG_DONTWAIT,(struct sockaddr *)&raw_host,sizeof(raw_host))==(int)size?0:-1;
}
static void raw_task(void *unused) {
    (void)unused;uint8_t bytes[1401];uint64_t next_status=0;
    memset(&raw_host,0,sizeof(raw_host));raw_host.sin_family=AF_INET;
    raw_host.sin_port=htons(CONFIG_SANITY_DATAGRAM_HOST_PORT);
    if(inet_pton(AF_INET,CONFIG_SANITY_DATAGRAM_HOST,&raw_host.sin_addr)!=1)abort();
    for(;;){
        xEventGroupWaitBits(wifi_events,1,pdFALSE,pdTRUE,portMAX_DELAY);
        raw_socket=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
        struct sockaddr_in local={.sin_family=AF_INET,.sin_port=htons(CONFIG_SANITY_DATAGRAM_PORT),.sin_addr.s_addr=htonl(INADDR_ANY)};
        if(raw_socket<0 || bind(raw_socket,(struct sockaddr *)&local,sizeof(local))<0){if(raw_socket>=0)close(raw_socket);vTaskDelay(pdMS_TO_TICKS(1000));continue;}
        fcntl(raw_socket,F_SETFL,O_NONBLOCK);
        uint64_t epoch=((uint64_t)esp_random()<<32)|esp_random();if(!epoch)epoch=1;
        portENTER_CRITICAL(&lock);sz_init(&state,CONFIG_SANITY_DEVICE_ID,OUTPUT_MODE,epoch);command_rx_us=0;portEXIT_CRITICAL(&lock);
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
        sz_diag_epoch(epoch);
#endif
        while(xEventGroupGetBits(wifi_events)&1){
            for(unsigned budget=0;budget<32;++budget){
                struct sockaddr_in source;socklen_t length=sizeof(source);
                int n=recvfrom(raw_socket,bytes,sizeof(bytes),0,(struct sockaddr *)&source,&length);
                if(n<0)break;
                if(source.sin_addr.s_addr!=raw_host.sin_addr.s_addr || source.sin_port!=raw_host.sin_port)continue;
                if(n==SZ_COMMAND_SIZE)command_bytes(bytes,(size_t)n,esp_timer_get_time());
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
                else if(n==SZ_PROBE_SIZE)sz_diag_probe_bytes(bytes,(size_t)n);
#endif
            }
            uint64_t now=esp_timer_get_time();
            if(now>=next_status){
                uint8_t status[SZ_STATUS_SIZE];portENTER_CRITICAL(&lock);sz_status(&state,now_ms(),fc_ready,voltage_mv,fc_age,status);portEXIT_CRITICAL(&lock);
                raw_send(status,sizeof(status),NULL);next_status=now+20000;
            }

            publish_health();publish_telemetry_to(raw_send,NULL);
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
            sz_diag_flush_pongs_to(raw_send,NULL);sz_diag_flush_traces_to(raw_send,NULL);
#endif
            vTaskDelay(1);
        }
        stop(4);close(raw_socket);raw_socket=-1;
    }
}
#endif

void app_main(void) {
#ifdef CONFIG_SANITY_MSP_TUNING
    sz_tune_init(&tuning);
#endif
    if (sz_usb_passthrough_selected()) {
        sz_usb_passthrough_run();
        return;
    }
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
    sz_diag_init();
#endif
    telemetry_events=xQueueCreate(64,sizeof(struct sz_fc_record));
    controller_telemetry_event=xQueueCreate(1,sizeof(struct sz_fc_record));
    if (!telemetry_events || !controller_telemetry_event) abort();
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_health.boot_id=((uint64_t)esp_random()<<32)|esp_random();
    esp_health.reset_reason=esp_reset_reason();
    nvs_handle_t health_nvs;
    if (nvs_open("sanity_health",NVS_READWRITE,&health_nvs)==ESP_OK) {
        uint32_t count=0;nvs_get_u32(health_nvs,"boots",&count);
        count=count==UINT32_MAX?1:count+1;
        if (nvs_set_u32(health_nvs,"boots",count)==ESP_OK && nvs_commit(health_nvs)==ESP_OK) esp_health.boot_count=count;
        nvs_close(health_nvs);
    }
    sz_init(&state, CONFIG_SANITY_DEVICE_ID, OUTPUT_MODE, 1);
    uart_start();
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
    se_runtime_start();
#endif
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
    // Keep UART polling off the Wi-Fi core; it preempts the low-priority ESKF.
    if (xTaskCreatePinnedToCore(output_task, "fc_output", 4096, NULL, 15, NULL, 1) != pdPASS)
        abort();
#else
    if (xTaskCreate(output_task, "fc_output", 4096, NULL, 15, NULL) != pdPASS)
        abort();
#endif
    wifi_start();
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
    sz_mocap_clock_udp_start(CONFIG_SANITY_DEVICE_ID, mocap_current_epoch);
#endif
    sz_hx711_start(esp_health.boot_id,esp_health.boot_count);
#ifdef CONFIG_SANITY_RAW_UDP
    if(xTaskCreate(raw_task,"unicast",8192,NULL,14,NULL)!=pdPASS)abort();
#else
#ifdef CONFIG_SANITY_SPLIT_TELEMETRY
    if(xTaskCreate(telemetry_task,"fc_telemetry",ZENOH_OWNER_STACK,NULL,4,NULL)!=pdPASS) abort();
#endif
    if (xTaskCreate(zenoh_task, "zenoh", ZENOH_OWNER_STACK, NULL, 5, NULL) != pdPASS)
        abort();
#endif
}
