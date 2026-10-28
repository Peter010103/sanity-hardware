import copy
import contextlib
from unittest.mock import patch
import importlib.util
import io
import json
import math
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace
import unittest

import calibrate_onboard_tension as cal


def packet(seq=1, raw=123, **changes):
    p=dict(schema=cal.SCHEMA, device_id=9, boot_id="1234567890abcdef",seq=seq,
           ready_us=seq*12500+1000,read_us=seq*12500+1200,raw=raw,flags=0,
           max_high_us=4,send_drops=0,resets=1,dout_gpio=1,sck_gpio=2,channel="A",gain=128)
    p.update(changes)
    return p


def session(scale=20000, intercept=123456, hysteresis=0.0, creep=0.0):
    result=dict(identity=dict(sensor_id="synthetic-only",device_id=9),gravity_m_s2=cal.G0,
                base_mass_g=70.0, points=[])
    for i,step in enumerate(cal.schedule([0,5,10,20],[7,17],2)):
        force=0 if step["kind"]=="unloaded" else (.070+step["added_g"]/1000)*cal.G0
        offset=hysteresis*scale if step["kind"]=="descending" else 0
        n=12
        samples=[packet(seq=i*100+k+1, raw=round(intercept+scale*force+offset+((k-(n-1)/2)*creep*scale if step["kind"]=="creep" else 0))) for k in range(n)]
        result["points"].append(dict(step,force_N=force,packets=samples,**cal.statistics_for(samples)))
    return result


class CalibrationTests(unittest.TestCase):
    def test_absolute_tension_includes_drone(self):
        s=session();fit=cal.fit_session(s)
        self.assertAlmostEqual(fit["counts_per_N"],20000,delta=2)
        self.assertAlmostEqual(fit["zero_counts"],123456,delta=1)
        baseline=next(p for p in s["points"] if p["kind"]=="ascending")
        self.assertAlmostEqual((baseline["mean_counts"]-fit["zero_counts"])/fit["counts_per_N"],.070*cal.G0,places=4)

    def test_negative_sensor_direction(self):
        fit=cal.fit_session(session(-20000))
        self.assertAlmostEqual(fit["counts_per_N"],-20000,delta=2)
        self.assertLess(fit["max_check_error_N"],1e-4)

    def test_hysteresis_reported_not_fitted_out(self):
        fit=cal.fit_session(session(hysteresis=.015))
        self.assertAlmostEqual(fit["max_hysteresis_N"],.015,delta=1e-4)
        self.assertAlmostEqual(fit["counts_per_N"],20000,delta=2)

    def test_check_weight_held_out(self):
        s=session()
        for p in s["points"]:
            if p["kind"]=="check":p["mean_counts"]+=2000
        fit=cal.fit_session(s)
        self.assertAlmostEqual(fit["counts_per_N"],20000,delta=2)
        self.assertAlmostEqual(fit["max_check_error_N"],.1,delta=1e-4)

    def test_creep_drift_retained(self):
        fit=cal.fit_session(session(creep=.001))
        self.assertTrue(any(abs(p["drift_N_per_s"])>.05 for p in fit["points"] if p["kind"]=="creep"))

    def test_no_response_rejected(self):
        with self.assertRaisesRegex(ValueError,"too small"):
            cal.fit_session(session(scale=0))

    def test_schedule_has_unloaded_zeros_and_descents(self):
        steps=cal.schedule([0,5,10],[7],2)
        self.assertEqual(steps[0]["kind"],"unloaded")
        self.assertEqual(steps[-1]["kind"],"unloaded")
        self.assertEqual([p["added_g"] for p in steps if p["cycle"]==1 and p["kind"]=="descending"],[5,0])
        for added in ([5,10,20],[0,0,10],[0,5]):
            with self.assertRaises(ValueError):cal.schedule(added,[],2)

    def test_decode_valid_signed(self):
        self.assertEqual(cal.decode_packet(json.dumps(packet(raw=-200)),9)["raw"],-200)

    def test_decode_faults_and_malformed(self):
        for changes in (dict(raw=8388607),dict(raw=-8388608),dict(max_high_us=60),dict(ready_us=-1),dict(gain=64),dict(dout_gpio=3),dict(raw=True),dict(flags=16)):
            with self.assertRaises(ValueError):cal.decode_packet(json.dumps(packet(**changes)),9)
        for data in ('[]','null','{}'):
            with self.assertRaises(ValueError):cal.decode_packet(data,9)
        self.assertEqual(cal.decode_packet(json.dumps(packet(raw=8388607,flags=2)),9)["flags"],2)

    def test_udp_freshness_boot_and_gaps(self):
        args=SimpleNamespace(device_id=9,source_ip="127.0.0.1",bind="127.0.0.1",port=0)
        raw=io.StringIO();stream=cal.UDPStream(args,raw)
        sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        address=stream.socket.getsockname()
        try:
            for p in (packet(1),packet(1),packet(4)):
                sender.sendto(json.dumps(p).encode(),address)
            self.assertEqual(stream.next_valid()["seq"],1)
            self.assertEqual(stream.next_valid()["seq"],4)
            self.assertEqual(stream.gaps,2)
            self.assertEqual(stream.duplicates,1)
            sender.sendto(json.dumps(packet(5,boot_id="fedcba0987654321")).encode(),address)
            with self.assertRaisesRegex(RuntimeError,"restarted"):stream.next_valid()
        finally:
            stream.socket.close();sender.close()

    def test_udp_flagged_is_not_force(self):
        args=SimpleNamespace(device_id=9,source_ip="127.0.0.1",bind="127.0.0.1",port=0)
        stream=cal.UDPStream(args,io.StringIO())
        sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            sender.sendto(json.dumps(packet(1,flags=2,raw=8388607)).encode(),stream.socket.getsockname())
            sender.sendto(json.dumps(packet(2)).encode(),stream.socket.getsockname())
            self.assertEqual(stream.next_valid()["seq"],2)
            self.assertEqual(stream.flagged,1)
        finally:
            stream.socket.close();sender.close()

    def test_stats_source_clock_not_host_spacing(self):
        s=cal.statistics_for([packet(k) for k in range(1,81)])
        self.assertAlmostEqual(s["sample_rate_hz"],80)
        self.assertAlmostEqual(s["max_interval_ms"],12.5)


class ResponseModelTests(unittest.TestCase):
    def test_quick_run_has_matching_loading_and_unloading(self):
        steps=cal.schedule([0,5,10,15,40],[],1,quick=True)
        self.assertEqual([p["added_g"] for p in steps if p["kind"]=="ascending"],[0,5,10,15,40])
        self.assertEqual([p["added_g"] for p in steps if p["kind"]=="descending"],[15,10,5,0])
        self.assertEqual(steps[0]["kind"],"unloaded")
        self.assertEqual(steps[-1]["kind"],"unloaded")

    def test_quadratic_response_is_evaluated_on_held_out_levels(self):
        s=session()
        for p in s["points"]:
            f=p["force_N"];p["mean_counts"]=200000-10000*f-20000*f*f
        review=cal.response_model_review(s)
        models={m["degree"]:m for m in review["branches"]["ascending"]}
        self.assertLess(models[2]["leave_one_force_level_out_RMS_counts"],1e-7)
        self.assertGreater(models[1]["leave_one_force_level_out_RMS_counts"],100)
        self.assertTrue(models[2]["monotonic_over_tested_range"])
        self.assertIn("diagnostic only",review["status"])

    def test_good_polynomial_fit_can_still_be_noninvertible(self):
        s=session()
        for p in s["points"]:
            f=p["force_N"];p["mean_counts"]=200000+10000*(f-.7)**2
        review=cal.response_model_review(s)
        quadratic=next(m for m in review["branches"]["ascending"] if m["degree"]==2)
        self.assertAlmostEqual(quadratic["training_R_squared"],1)
        self.assertFalse(quadratic["monotonic_over_tested_range"])

    def test_direction_dependence_kept_in_raw_counts(self):
        s=session()
        for p in s["points"]:
            p["mean_counts"]=200000-10000*p["force_N"]+(1000 if p["kind"]=="descending" else 0)
        review=cal.response_model_review(s)
        self.assertTrue(review["hysteresis_counts"])
        self.assertEqual({p["unloading_minus_loading_counts"] for p in review["hysteresis_counts"]},{1000})


class RecoveryTests(unittest.TestCase):
    def replay(self, frames, duration=1.0, settle_s=0.0, recovery_s=2.0):
        clock=SimpleNamespace(now=0.0)
        args=SimpleNamespace(device_id=9,source_ip="127.0.0.1",bind="127.0.0.1",port=0)
        stream=cal.UDPStream(args,io.StringIO())
        stream.socket.close()
        class ReplaySocket:
            def recvfrom(self, size):
                clock.now+=.1
                if not frames:
                    raise socket.timeout()
                frame=frames.pop(0)
                if frame is None:
                    raise socket.timeout()
                return json.dumps(frame).encode(),("127.0.0.1",7460)
        stream.socket=ReplaySocket()
        with patch.object(cal.time,"monotonic",side_effect=lambda:clock.now),contextlib.redirect_stdout(io.StringIO()):
            packets=stream.recovering_window(duration,"ascending",settle_s=settle_s,recovery_s=recovery_s)
        return packets,stream

    def frames(self, count, **changes):
        return [packet(seq=i,ready_us=i*100000,read_us=i*100000+100,**changes) for i in range(1,count+1)]

    def test_clean_ten_hz_uses_source_duration(self):
        points,stream=self.replay(self.frames(20,raw=100))
        self.assertGreaterEqual(points[-1]["ready_us"]-points[0]["ready_us"],1000000)
        self.assertLess(len(points),20)
        self.assertEqual(stream.capture_diagnostics["discarded_windows"],0)

    def test_timing_fault_and_settling_discard_entire_old_fragment(self):
        frames=self.frames(40,raw=222)
        for frame in frames[:5]:frame["raw"]=99999
        frames[5].update(raw=-8388608,flags=12,max_high_us=47,resets=2)
        for frame in frames[6:10]:frame.update(raw=88888,flags=8,resets=2)
        for frame in frames[10:]:frame["resets"]=2
        points,stream=self.replay(frames,settle_s=.2)
        self.assertEqual({p["raw"] for p in points},{222})
        self.assertGreaterEqual(points[0]["ready_us"],1100000)
        self.assertEqual(stream.capture_diagnostics["flagged_packets"],5)
        self.assertEqual(stream.capture_diagnostics["discarded_windows"],1)
        self.assertIn('"flags": 12',stream.file.getvalue())

    def test_lost_fault_packet_detected_from_reset_counter(self):
        frames=self.frames(30,raw=222)
        for frame in frames[:5]:frame["raw"]=99999
        for frame in frames[5:]:frame["resets"]=2
        points,stream=self.replay(frames,settle_s=.2)
        self.assertEqual({p["raw"] for p in points},{222})
        self.assertTrue(any(e.get("reset_counter_changed") for e in stream.capture_diagnostics["recovery_events"]))

    def test_persistent_saturation_cannot_produce_calibration_point(self):
        with self.assertRaises(cal.ADCRecoveryTimeout):
            self.replay(self.frames(40,raw=8388607,flags=2),recovery_s=.5)

    def test_healthy_stream_gap_discards_earlier_fragment(self):
        frames=self.frames(35,raw=222)
        for frame in frames[:5]:frame["raw"]=99999
        frames=frames[:5]+[None]*12+frames[5:]
        points,stream=self.replay(frames,recovery_s=4)
        self.assertEqual({p["raw"] for p in points},{222})
        self.assertTrue(any(e.get("reason")=="healthy stream gap >1s" for e in stream.capture_diagnostics["recovery_events"]))

    def test_source_reboot_is_not_silently_recovered(self):
        frames=self.frames(30)
        frames[5]["boot_id"]="fedcba0987654321"
        with self.assertRaisesRegex(RuntimeError,"restarted"):
            self.replay(frames)


if __name__=="__main__":
    unittest.main()
