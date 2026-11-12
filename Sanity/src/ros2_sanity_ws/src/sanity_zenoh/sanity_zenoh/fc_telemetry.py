"""Read-only FC telemetry; timestamps describe ESP observations, not FC sampling."""

import math
import struct
from sanity_zenoh.controller_status import decode_wrapped as decode_controller_status
from sanity_zenoh.paper_status import COMMAND as PAPER_COMMAND, decode_wrapped as decode_paper_status
from sanity_zenoh.controller_timing import COMMAND as TIMING_COMMAND, decode_wrapped as decode_controller_timing

BATCH_HEADER = struct.Struct(">4sBBHI")
RECORD_HEADER = struct.Struct(">BBBBQIIQQQ")
SANITY_GYRO_DPS_PER_COUNT = 2000. / 32768.


def decode_fc_telemetry(payload, expected_device=None, rpm_kT=None, gyro_dps_per_count=SANITY_GYRO_DPS_PER_COUNT):
    if len(payload) < BATCH_HEADER.size or len(payload) > 1400:
        raise ValueError("FC telemetry batch length")
    magic, version, device, count, dropped = BATCH_HEADER.unpack_from(payload)
    if magic != b"SZF1" or version != 1 or not 1 <= device <= 255 or not 1 <= count <= 31:
        raise ValueError("FC telemetry batch header")
    if expected_device is not None and device != expected_device:
        raise ValueError("FC telemetry device mismatch")
    if rpm_kT is not None and (not math.isfinite(rpm_kT) or rpm_kT <= 0):
        raise ValueError("Positive finite RPM thrust coefficient required")
    if not math.isfinite(gyro_dps_per_count) or gyro_dps_per_count <= 0:
        raise ValueError("Positive finite FC gyro scale required")
    records = []
    cursor = BATCH_HEADER.size
    for _ in range(count):
        if cursor + RECORD_HEADER.size > len(payload):
            raise ValueError("Truncated FC telemetry record")
        kind, command, flags, size, epoch, session, sequence, first, start, end = RECORD_HEADER.unpack_from(payload, cursor)
        cursor += RECORD_HEADER.size
        if kind not in (1, 2, 3, 4, 5) or flags & ~3 or size > 128 or cursor + size > len(payload):
            raise ValueError("Invalid FC telemetry record")
        raw = payload[cursor:cursor + size]
        cursor += size
        if (start and end < start) or (first and end < first):
            raise ValueError("FC telemetry timestamp order")
        record = {"type":kind,"msp_command":command,"epoch":epoch,"flags":flags,"payload_hex":raw.hex()}
        if kind == 1:
            record.update(command_session=session,command_sequence=sequence,command_received_esp_us=first,
                          uart_write_begin_esp_us=start,uart_write_return_esp_us=end)
        else:
            record.update(response_first_observed_esp_us=first or None,response_parsed_esp_us=end,
                          query_write_begin_esp_us=start or None,query_association="same MSP command; no FC transaction ID" if flags & 2 else None)
        try:
            if kind == 5:
                if command or len(raw)!=48:
                    raise ValueError("ESP health record length/command")
                boot, count, reason, free, minimum, disconnects, connects, last_reason, last_us, rssi, channel = struct.unpack(">Q7IQbB2x",raw)
                names=("UNKNOWN","POWERON","EXT","SW","PANIC","INT_WDT","TASK_WDT","WDT","DEEPSLEEP","BROWNOUT","SDIO","USB","JTAG","EFUSE","PWR_GLITCH","CPU_LOCKUP")
                record.update(esp_health={"boot_id":boot,"boot_count":count or None,"reset_reason_code":reason,
                    "reset_reason":names[reason] if reason<len(names) else "UNRECOGNIZED","uptime_us":end,
                    "free_heap_bytes":free,"minimum_free_heap_bytes":minimum,"wifi_disconnects":disconnects,"wifi_connections":connects,
                    "last_wifi_disconnect_reason":last_reason or None,"last_wifi_disconnect_esp_us":last_us or None,"wifi_rssi_dBm":rssi,"wifi_channel":channel,
                    "esp_supply_voltage_V":None,"power_scope":"Reset diagnostics only; ESP supply rail is not measured"})
            elif kind == 1:
                if command == 200 and len(raw) == 16:
                    record["transmitted_rc_channels"] = list(struct.unpack("<8H", raw))
                    record["rc_channel_order"] = ["roll","pitch","throttle","yaw","arm","acro","aux3","aux4"]
                elif command == 214 and len(raw) == 8:
                    record["transmitted_motor_test_cache"] = list(struct.unpack("<4H", raw))
                else:
                    raise ValueError("Unexpected MSP output length/command")
            elif kind == 3:
                record["msp_error"] = True
            elif kind == 4:
                if raw:
                    raise ValueError("Timeout record has payload")
                record["query_timed_out"] = True
            elif command == 102:
                if len(raw) != 18:
                    raise ValueError("MSP_RAW_IMU requires 18 bytes")
                values = struct.unpack("<9h", raw)
                record.update(accelerometer_counts_fc=list(values[:3]),gyro_counts_fc=list(values[3:6]),
                              gyro_dps_fc=[x*gyro_dps_per_count for x in values[3:6]],
                              gyro_rad_s_fc=[math.radians(x*gyro_dps_per_count) for x in values[3:6]],
                              gyro_scale_dps_per_count=gyro_dps_per_count,
                              gyro_scale_provenance="Recorded Sanity ICM42688P/BF4.5.1, GYRO_SCALE_2000DPS; override for other sensor configurations",
                              magnetometer_counts_fc=list(values[6:]),
                              frame="Betaflight-reported axes; no model-frame transform applied")
            elif command == 104:
                if len(raw) != 16:
                    raise ValueError("MSP_MOTOR requires eight uint16 outputs")
                record["motor_outputs_external_fc"] = list(struct.unpack("<8H", raw))
            elif command == 105:
                if len(raw) < 8 or len(raw) % 2:
                    raise ValueError("MSP_RC requires complete uint16 channels")
                record["received_rc_channels_fc"] = list(struct.unpack("<" + "H" * (len(raw)//2), raw))
            elif command == 139:
                if not raw or not 1 <= raw[0] <= 8 or len(raw) != 1 + 13 * raw[0]:
                    raise ValueError("MSP_MOTOR_TELEMETRY motor count/length")
                motors = []
                for index in range(raw[0]):
                    rpm, invalid, temperature, voltage, current, consumption = struct.unpack_from("<IHBHHH", raw, 1+13*index)
                    motors.append({"motor_output":index+1,"rpm":rpm,"invalid_packet_percent":invalid/100,
                                   "rpm_quality_usable":rpm > 0 and invalid < 10000,"temperature_C":temperature,
                                   "esc_voltage_raw":voltage,"esc_current_raw":current,"consumption_mAh":consumption})
                record["motors"] = motors
                record["estimated_collective_N"] = (rpm_kT * sum((m["rpm"] * 2 * math.pi / 60)**2 for m in motors)
                    if rpm_kT is not None and len(motors) == 4 and all(m["rpm_quality_usable"] for m in motors) else None)
                record["thrust_estimate_scope"] = "RPM/static kT estimate; quality-usable four-motor samples only; not measured thrust"
            elif command == 130:
                if len(raw) != 11:
                    raise ValueError("MSP_BATTERY_STATE requires 11 bytes")
                record.update(battery_cells=raw[0],voltage_V=struct.unpack_from("<H",raw,9)[0]/100,
                              current_A=struct.unpack_from("<h",raw,6)[0]/100,consumption_mAh=struct.unpack_from("<H",raw,4)[0])
            elif command in (94,95,112,150,202,250):
                record["tuning_msp_payload_hex"] = raw.hex()
            elif command == 255:
                if len(raw) >= 5 and struct.unpack_from('<H',raw,1)[0] == PAPER_COMMAND:
                    record['paper_state'] = decode_paper_status(raw)
                elif len(raw) >= 5 and struct.unpack_from('<H',raw,1)[0] == 0x7f15:
                    record['paper_transport_acknowledged'] = True
                elif len(raw) >= 5 and struct.unpack_from('<H',raw,1)[0] == TIMING_COMMAND:
                    record['controller_timing'] = decode_controller_timing(raw)
                else:
                    record['controller_state'] = decode_controller_status(raw)
            elif command == 200:
                if raw:
                    raise ValueError("MSP_SET_RAW_RC acknowledgment must be empty")
                record["rc_write_acknowledged"] = True
            else:
                raise ValueError("Unexpected FC telemetry MSP command")
        except (ValueError, struct.error) as error:
            record["decode_error"] = str(error)
        records.append(record)
    if cursor != len(payload):
        raise ValueError("Trailing FC telemetry bytes")
    return {"wire_version":1,"device":device,"esp_dropped_records":dropped,"records":records,
            "timestamp_scope":"ESP monotonic microseconds; MSP request/parse times are not FC sensor acquisition times"}
