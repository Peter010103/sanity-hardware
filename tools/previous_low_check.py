"""One finite 1250-count collective plateau using the installed bench engine."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import signal
import os
import subprocess
import sys

from stand_config import ROOT as HERE, FC_PORT, FC_UID, FC_PROFILE

import bench_experiments as b
import betaflight_msp as bf

EXPECTED_UID = FC_UID
PORT = FC_PORT
PLAN = [dict(name='collective_1250_low_check', commands=[1250]*4,
             duration_s=3.0, role='comparison', cycle=0)]


class RelativeBaselineBench(b.Bench):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Historical zero is not valid on the rewired rig; only delta is physical.
        # Retain ADC rails, finite checks and the independent +/-0.8 N delta guard.
        self.force_range = (-float('inf'), float('inf'))

    def poll_rpm(self):
        super().poll_rpm()
        import struct
        start, end, payload = self.query(104)
        actual = list(struct.unpack_from('<4H', payload))
        self.log.event('motor_output_readback', request_start_ns=start,
                       reply_ns=end, requested=list(self.vector), actual=actual)
        if self.vector != b.STOP and end-self.command_ack_ns > 100_000_000 and actual != list(self.vector):
            raise RuntimeError('FC motor-output readback does not match the bounded command')

    def poll_battery(self):
        super().poll_battery()
        data = self.latest_battery
        if data['detected_cells'] != 2 or not 7.52 <= data['voltage_V'] <= 8.1125:
            raise RuntimeError('Battery outside the old model comparison envelope')
        if data['current_A'] > 8.0:
            raise RuntimeError('FC current reading exceeds conservative 8 A test cutoff')


def priority_sensor():
    allowed = os.sched_getaffinity(0)
    cpu = max(allowed)
    os.sched_setaffinity(0, {cpu})
    os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
    print('ADC worker: SCHED_FIFO priority 10, CPU '+str(cpu), flush=True)
    return b.NativeHX711(timeout=.25, timing_error=b.TimingError)


class PriorityForceReader(b.ForceReader):
    def __init__(self):
        super().__init__(factory=priority_sensor)


def snapshot():
    profile = json.loads(FC_PROFILE.read_text())
    t = bf.USBSerial(PORT)
    try:
        c = bf.MSP(t, timeout=.5)
        live = bf.inspect_controller(c)
        b.validate_setup(live, profile)
        if EXPECTED_UID and live['uid_payload_hex'] != EXPECTED_UID:
            raise RuntimeError('Unexpected FC UID')
        boxes = list(c.query(119))
        state = bf.status(c.query(101), boxes)
        motors = bf.motor_telemetry(c.query(139))
        import struct
        outputs = list(struct.unpack_from('<4H', c.query(104)))
        return dict(controller=live, status=state, battery=bf.battery(c.query(130)),
                    motors=motors, outputs=outputs)
    finally:
        t.close()
