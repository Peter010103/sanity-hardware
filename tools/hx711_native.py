"""Native transfer loop for verified P3768 / Tegra234 header pins 29 and 31."""
import ctypes as C
import os
from pathlib import Path

LIBRARY = Path(__file__).with_name("libhx711.so")


class Sample(C.Structure):
    _fields_ = [("raw_counts", C.c_int32), ("ready_ns", C.c_uint64),
                ("complete_ns", C.c_uint64), ("unix_ns", C.c_uint64),
                ("max_high_bound_ns", C.c_uint64)]


def supported_board():
    try:
        compatible = Path("/proc/device-tree/compatible").read_bytes().split(b"\0")
    except OSError:
        return False
    return b"nvidia,tegra234" in compatible and any(
        part.startswith(b"nvidia,p3768-") for part in compatible)


class NativeHX711:
    def __init__(self, timeout=2.0, timing_error=RuntimeError):
        if not supported_board():
            raise RuntimeError("Native backend requires verified P3768/Tegra234 header mapping")
        self.timeout, self.timing_error = timeout, timing_error
        self.handle = None
        self.lib = C.CDLL(str(LIBRARY), use_errno=True)
        self.lib.hx_open.argtypes = [C.c_char_p, C.c_uint, C.c_uint]
        self.lib.hx_open.restype = C.c_void_p
        self.lib.hx_close.argtypes = [C.c_void_p]
        self.lib.hx_close.restype = None
        self.lib.hx_reset.argtypes = [C.c_void_p]
        self.lib.hx_reset.restype = C.c_int
        self.lib.hx_read.argtypes = [C.c_void_p, C.c_uint64, C.POINTER(Sample)]
        self.lib.hx_read.restype = C.c_int
        self.lib.hx_chip_matches.argtypes = [C.c_char_p]
        self.lib.hx_chip_matches.restype = C.c_int
        chips = sorted(Path("/dev").glob("gpiochip*"))
        matching = [p for p in chips if self.lib.hx_chip_matches(str(p).encode())]
        if len(matching) != 1:
            raise RuntimeError("Cannot uniquely identify tegra234-gpio device")
        self.chip_path = str(matching[0])
        # NVIDIA BOARD mapping for P3768: PQ.05=line105; PQ.06=line106.
        self.handle = self.lib.hx_open(self.chip_path.encode(), 105, 106)
        if not self.handle:
            code = C.get_errno()
            raise OSError(code, os.strerror(code), self.chip_path)

    def reset(self):
        if self.lib.hx_reset(self.handle) < 0:
            code = C.get_errno()
            raise OSError(code, os.strerror(code))

    def read(self):
        sample = Sample()
        status = self.lib.hx_read(self.handle, int(self.timeout * 1e9), C.byref(sample))
        if status == -2:
            raise TimeoutError("HX711 DOUT stayed HIGH: check clock-pin pinmux, power and wiring")
        if status == -3:
            raise self.timing_error("Clock pulse software bound %.1f us exceeds 50 us; sample rejected" %
                                    (sample.max_high_bound_ns / 1000))
        if status == -4:
            raise RuntimeError("DOUT did not return HIGH after pulse 25; sample rejected")
        if status != 0:
            code = C.get_errno()
            raise OSError(code, os.strerror(code))
        return {"raw_counts": sample.raw_counts,
                "ready_observed_monotonic_ns": sample.ready_ns,
                "read_complete_monotonic_ns": sample.complete_ns,
                "unix_time_ns": sample.unix_ns,
                "max_clock_high_bound_us": sample.max_high_bound_ns / 1000}

    def close(self):
        if self.handle:
            self.lib.hx_close(self.handle)
            self.handle = None
