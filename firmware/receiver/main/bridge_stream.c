#include "bridge_stream.h"

int sz_bridge_pump(struct sz_bridge_stream *stream, sz_bridge_read read_bytes,
                   sz_bridge_write write_bytes) {
    if (stream->sent == stream->length) {
        int count = read_bytes(stream->bytes, sizeof(stream->bytes));
        if (count < 0 || (size_t)count > sizeof(stream->bytes))
            return -1;
        stream->length = (size_t)count;
        stream->sent = 0;
    }
    if (stream->sent == stream->length)
        return 0;
    size_t remaining = stream->length - stream->sent;
    int count = write_bytes(stream->bytes + stream->sent, remaining);
    if (count < 0 || (size_t)count > remaining)
        return -1;
    stream->sent += (size_t)count;
    return count;
}
