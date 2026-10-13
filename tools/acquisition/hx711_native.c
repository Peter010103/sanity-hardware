/* HX711 channel A / gain 128 via Linux GPIO character-device ABI v1.
 * Build: cc -O2 -Wall -Wextra -Werror -shared -fPIC -o libhx711.so hx711_native.c
 * No automatic pinmux changes; no hard real-time guarantee.
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

struct hx_device { int dout_fd; int sck_fd; };
struct hx_sample {
    int32_t raw_counts;
    uint64_t ready_ns, complete_ns, unix_ns, max_high_bound_ns;
};

int hx_chip_matches(const char *chip_path) {
    int chip = open(chip_path, O_RDONLY | O_CLOEXEC);
    if (chip < 0) return 0;
    struct gpiochip_info info = {0};
    int result = ioctl(chip, GPIO_GET_CHIPINFO_IOCTL, &info);
    close(chip);
    return result == 0 && !strcmp(info.label, "tegra234-gpio");
}

static uint64_t now_ns(clockid_t clock) {
    struct timespec t;
    clock_gettime(clock, &t);
    return (uint64_t)t.tv_sec * 1000000000ULL + (uint64_t)t.tv_nsec;
}
static void sleep_ns(long ns) {
    struct timespec t = {ns / 1000000000L, ns % 1000000000L};
    while (nanosleep(&t, &t) < 0 && errno == EINTR) {}
}
static void short_delay(void) {
    uint64_t end = now_ns(CLOCK_MONOTONIC) + 1000;
    while (now_ns(CLOCK_MONOTONIC) < end) {}
}
static int set_clock(struct hx_device *d, int value) {
    struct gpiohandle_data data = {0};
    data.values[0] = (unsigned char)value;
    return ioctl(d->sck_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
}
static int get_data(struct hx_device *d) {
    struct gpiohandle_data data = {0};
    if (ioctl(d->dout_fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data) < 0) return -1;
    return !!data.values[0];
}
static int request_line(int chip, unsigned line, unsigned flags) {
    struct gpiohandle_request request = {0};
    request.lineoffsets[0] = line;
    request.flags = flags;
    request.lines = 1;
    strcpy(request.consumer_label, "sanity-hx711");
    if (ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, &request) < 0) return -1;
    return request.fd;
}
void hx_close(struct hx_device *d) {
    if (!d) return;
    if (d->sck_fd >= 0) { set_clock(d, 0); close(d->sck_fd); }
    if (d->dout_fd >= 0) close(d->dout_fd);
    free(d);
}
int hx_reset(struct hx_device *d) {
    if (set_clock(d, 1) < 0) return -1;
    sleep_ns(1000000);
    if (set_clock(d, 0) < 0) return -1;
    sleep_ns(500000000);
    return 0;
}
struct hx_device *hx_open(const char *chip_path, unsigned dout, unsigned sck) {
    int chip = open(chip_path, O_RDONLY | O_CLOEXEC);
    if (chip < 0) return NULL;
    struct gpiochip_info info = {0};
    if (ioctl(chip, GPIO_GET_CHIPINFO_IOCTL, &info) < 0) { close(chip); return NULL; }
    if (strcmp(info.label, "tegra234-gpio") || dout >= info.lines || sck >= info.lines || dout == sck) {
        close(chip); errno = EINVAL; return NULL;
    }
    struct hx_device *d = calloc(1, sizeof(*d));
    if (!d) { close(chip); return NULL; }
    d->sck_fd = d->dout_fd = -1;
    d->sck_fd = request_line(chip, sck, GPIOHANDLE_REQUEST_OUTPUT);
    if (d->sck_fd >= 0) d->dout_fd = request_line(chip, dout, GPIOHANDLE_REQUEST_INPUT);
    int error = errno;
    close(chip);
    if (d->sck_fd < 0 || d->dout_fd < 0) { hx_close(d); errno = error; return NULL; }
    if (hx_reset(d) < 0) { error = errno; hx_close(d); errno = error; return NULL; }
    return d;
}
/* 0=success, -1=system call, -2=ready timeout, -3=clock timing, -4=stuck data. */
int hx_read(struct hx_device *d, uint64_t timeout_ns, struct hx_sample *s) {
    memset(s, 0, sizeof(*s));
    uint64_t deadline = now_ns(CLOCK_MONOTONIC) + timeout_ns;
    for (;;) {
        int bit = get_data(d);
        if (bit < 0) return -1;
        if (!bit) break;
        if (now_ns(CLOCK_MONOTONIC) >= deadline) return -2;
        sleep_ns(1000000);
    }
    s->ready_ns = now_ns(CLOCK_MONOTONIC);
    short_delay();
    uint32_t word = 0;
    for (int i = 0; i < 25; ++i) {
        uint64_t begin = now_ns(CLOCK_MONOTONIC);
        if (set_clock(d, 1) < 0) { int error = errno; set_clock(d, 0); errno = error; return -1; }
        short_delay();
        if (set_clock(d, 0) < 0) return -1;
        uint64_t bound = now_ns(CLOCK_MONOTONIC) - begin;
        if (bound > s->max_high_bound_ns) s->max_high_bound_ns = bound;
        if (bound > 50000) return -3;
        short_delay();
        int bit = get_data(d);
        if (bit < 0) return -1;
        if (i < 24) word = (word << 1) | (unsigned)bit;
        else if (!bit) return -4;
    }
    s->raw_counts = (word & 0x800000U) ? (int32_t)word - 16777216 : (int32_t)word;
    s->complete_ns = now_ns(CLOCK_MONOTONIC);
    s->unix_ns = now_ns(CLOCK_REALTIME);
    return 0;
}
