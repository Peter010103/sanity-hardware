#pragma once
#include <stddef.h>
#include <stdint.h>

struct sz_bridge_stream {
    uint8_t bytes[512];
    size_t length;
    size_t sent;
};

typedef int (*sz_bridge_read)(void *, size_t);
typedef int (*sz_bridge_write)(const void *, size_t);
int sz_bridge_pump(struct sz_bridge_stream *stream, sz_bridge_read read_bytes,
                   sz_bridge_write write_bytes);
