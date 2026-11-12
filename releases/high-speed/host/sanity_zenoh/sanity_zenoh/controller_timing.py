"""SIT1 FC cumulative timing diagnostics, MSP2 0x7f14; never control authority.

Counters are boot-scoped uint32 values. Every invocation is counted on the FC,
independently of telemetry/Blackbox sampling. PID timing excludes the separately
measured gyro sample and filter tasks, and is not whole-scheduler utilization.
"""
import struct
from sanity_zenoh.controller_status import crc8

COMMAND = 0x7F14
SIZE = 80
FIELDS = ('fc_time_us', 'apply_sequence', 'nominal_period_us', 'completed_pid_cycles',
          'last_pid_execution_us', 'maximum_pid_execution_us', 'pid_execution_deadline_misses',
          'late_apply_intervals', 'pid_over_90_percent_budget', 'trace_log_dropped_bytes',
          'last_apply_interval_us', 'last_gyro_sample_execution_us', 'maximum_gyro_sample_execution_us',
          'last_gyro_filter_execution_us', 'maximum_gyro_filter_execution_us',
          'gyro_sample_overruns', 'gyro_filter_overruns')
COUNTERS = ('apply_sequence', 'completed_pid_cycles', 'pid_execution_deadline_misses',
            'late_apply_intervals', 'pid_over_90_percent_budget', 'trace_log_dropped_bytes',
            'gyro_sample_overruns', 'gyro_filter_overruns')


def decode(data):
    if len(data) != SIZE or data[:4] != b'SIT1' or data[4] != 1:
        raise ValueError('FC timing SIT1 length/magic/version')
    if data[5] & ~7 or any(data[6:8]) or any(data[76:80]):
        raise ValueError('FC timing SIT1 flags/reserved')
    result = dict(zip(FIELDS, struct.unpack_from('<17I', data, 8)))
    result.update(version=1, trace_requested=bool(data[5] & 1),
                  configured=bool(data[5] & 2), runtime_initialized=bool(data[5] & 4),
                  scope='Boot-scoped cumulative counters; PID task excludes separate gyro sample/filter tasks',
                  counter_wrap_bits=32, control_authority=False)
    return result


def decode_wrapped(payload):
    if len(payload) != SIZE + 6:
        raise ValueError('FC timing MSP2-over-v1 length')
    flags, command, length = struct.unpack_from('<BHH', payload)
    if flags or command != COMMAND or length != SIZE or crc8(payload[:-1]) != payload[-1]:
        raise ValueError('FC timing MSP2-over-v1 header/CRC')
    return decode(payload[5:-1])


def counter_deltas(previous, current):
    """Wrapped deltas only within one known FC boot and unchanged timing profile.

    A reboot can resemble uint32 wrap. The caller must establish boot continuity
    independently; these diagnostic snapshots deliberately grant no authority.
    """
    if previous['nominal_period_us'] != current['nominal_period_us']:
        raise ValueError('Cannot compare timing counters across period changes')
    return {key: (current[key] - previous[key]) & 0xFFFFFFFF for key in COUNTERS}
