#!/usr/bin/env python3
"""Inspect/fix only HX711 clock pin 31 on the verified P3768/Tegra234 setup.

Uses NVIDIA's documented runtime PADCTL procedure. Does not change boot files.
sudo python3 hx711_pinmux.py status|enable-clock|restore
"""
import argparse
import json
import mmap
import os
from pathlib import Path
import struct
import sys
import time

from hx711_native import supported_board

BASE, OFFSET = 0x02430000, 0x70
INPUT_VALUE, OUTPUT_VALUE = 0x58, 0x08
BACKUP = Path(__file__).with_name("pinmux_clock_original.json")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["status", "enable-clock", "restore"])
    args = parser.parse_args()
    if not supported_board():
        parser.error("Requires the verified P3768/Tegra234 header mapping")
    if os.geteuid() != 0:
        parser.error("Use sudo for access to the two documented PADCTL registers")
    write = args.action != "status"
    fd = os.open("/dev/mem", (os.O_RDWR if write else os.O_RDONLY) | os.O_SYNC)
    try:
        prot = mmap.PROT_READ | (mmap.PROT_WRITE if write else 0)
        with mmap.mmap(fd, 4096, flags=mmap.MAP_SHARED, prot=prot, offset=BASE) as mem:
            current = struct.unpack_from("<I", mem, OFFSET)[0]
            data_pad = struct.unpack_from("<I", mem, 0x68)[0]
            print("DOUT pin 29 pad = %s; SCK pin 31 pad = %s" % (hex(data_pad), hex(current)))
            if not write:
                print("Clock output enabled" if current == OUTPUT_VALUE else "Clock pad needs review")
                return
            if current not in (INPUT_VALUE, OUTPUT_VALUE):
                raise RuntimeError("Unexpected clock pad value; refusing to change it")
            if args.action == "enable-clock":
                target = OUTPUT_VALUE
                if current == target:
                    print("Already configured; no change")
                    return
                record = {"time_unix": time.time(), "board_pin": 31,
                          "address": "0x02430070", "before": hex(current), "after": hex(target),
                          "persistence": "current boot only"}
                if BACKUP.exists():
                    old = json.loads(BACKUP.read_text())
                    for key in ("board_pin", "address", "before", "after"):
                        if old[key] != record[key]:
                            raise RuntimeError("Existing pinmux backup differs; refusing to overwrite")
                else:
                    with BACKUP.open("x") as file:
                        json.dump(record, file, indent=2)
            else:
                record = json.loads(BACKUP.read_text())
                if (record["board_pin"] != 31 or record["address"] != "0x02430070" or
                        int(record["before"], 16) != INPUT_VALUE or
                        int(record["after"], 16) != OUTPUT_VALUE):
                    raise RuntimeError("Unexpected saved values; refusing to restore")
                target = INPUT_VALUE
                if current == target:
                    print("Already restored; no change")
                    return
            # Own this line exclusively before changing its pad. Start clock LOW.
            # A running reader must release it first; do not reconfigure underneath it.
            import Jetson.GPIO as GPIO
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(31, GPIO.OUT, initial=GPIO.LOW)
            try:
                struct.pack_into("<I", mem, OFFSET, target)
                actual = struct.unpack_from("<I", mem, OFFSET)[0]
                if actual != target:
                    raise RuntimeError("PADCTL readback differs from requested value")
                print("Clock pad changed: %s -> %s (current boot only)" % (hex(current), hex(actual)))
            finally:
                GPIO.output(31, GPIO.LOW)
                GPIO.cleanup(31)
    finally:
        os.close(fd)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print("ERROR: " + str(exc), file=sys.stderr)
        sys.exit(1)
