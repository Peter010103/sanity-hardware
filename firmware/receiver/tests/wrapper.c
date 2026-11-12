#include "bench.h"
#include "control.h"
static struct sz_state state;
static struct bench_state bench;
static uint16_t model_voltage=7800;
void test_set_voltage(unsigned mv) {model_voltage=(uint16_t)mv;}
void test_reset(unsigned mode, unsigned long long epoch) {
    sz_init(&state, 1, (uint8_t)mode, epoch);
    model_voltage=7800;
}
int test_receive(const unsigned char *packet, unsigned length, unsigned now) {
    struct sz_command command;
    return sz_decode(packet, length, &command) && sz_calibrate(&command,model_voltage) && sz_accept(&state, &command, now);
}
int test_output(unsigned now, unsigned short *values) { return sz_output(&state, now, values); }
int test_enabled(void) { return state.enabled; }
int test_latched(void) { return state.rearm_required; }
void test_status(unsigned now, unsigned char *out) { sz_status(&state, now, true, 7800, 10, out); }
void test_bench_reset(void) { bench_init(&bench); }
void test_bench_feed(const unsigned char *data, unsigned n, unsigned now) {
    for (unsigned i = 0; i < n; ++i)
        bench_byte(&bench, data[i], now);
}
int test_bench_ready(unsigned now) { return bench_ready(&bench, now); }
unsigned test_bench_voltage(unsigned now) {
    return (bench.seen & (1u << 8)) && now - bench.stamp[8] < 150 ? bench.voltage_mv : 0;
}

int test_receive_msp(const unsigned char *packet, unsigned length, unsigned now, int disarmed) {
    struct sz_command command;
    return sz_decode(packet, length, &command) && sz_calibrate(&command,model_voltage) && sz_accept_msp(&state, &command, now, disarmed);
}
int test_bench_ctbr_ready(unsigned now) { return bench_ctbr_ready(&bench, now); }
int test_decode_values(const unsigned char *packet, unsigned length, unsigned short *values) {
    struct sz_command command;
    if (!sz_decode(packet,length,&command) || !sz_calibrate(&command,model_voltage)) return 0;
    for (unsigned i=0;i<4;++i) values[i]=command.values[i];
    return 1;
}

int test_output_msp(unsigned now, unsigned char *out) {
    uint16_t values[4];
    sz_output(&state, now, values);
    return (int)sz_rc_msp_select(values, state.enabled, state.enabled && state.command.controller_selection && state.command.indi_requested, out);
}
int test_paper_frame(const unsigned char *p,unsigned n,unsigned now,unsigned char *out) {
    struct sz_command command;
    if(!sz_decode(p,n,&command))return 0;
    return (int)sz_paper_msp(&command,now,out);
}
int test_paper_capability(const unsigned char *p,unsigned n) {return sz_paper_capable(p,n);}
int test_paper_after_rc(const unsigned char *p,unsigned n,unsigned now,unsigned expected,int written,unsigned char *out) {
    struct sz_command command;
    if(!sz_decode(p,n,&command))return 0;
    return (int)sz_paper_after_rc(&command,now,expected,written,out);
}
static struct sz_paper_link paper_test_link;
void test_paper_observe(const unsigned char *p,unsigned n,unsigned epoch,unsigned now) {sz_paper_observe(&paper_test_link,epoch,now,p,n);}
void test_paper_claim(int enabled) {paper_test_link.negotiated=enabled;}
int test_paper_admit(unsigned epoch,unsigned now) {return sz_paper_admit(&paper_test_link,epoch,now);}
int test_paper_forward(unsigned epoch,int enabled) {return sz_paper_forward(&paper_test_link,epoch,enabled);}
