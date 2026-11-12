"""Protocol and fault-injection tests. No hardware or GPIO is accessed."""
import contextlib
import csv
import io
import json
import os
from pathlib import Path
import queue
import statistics
import struct
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import bench_experiments as be

CAL = dict(schema_version=1, channel="A", gain=128, counts_per_N=1000., zero_counts=0., measured_force_range_N=[0.,10.])
PROFILE = json.loads((be.HERE/"fc_profile.json").read_text())


def sample(raw=1000):
    t=be.mono()
    return dict(read_begin_ns=t-1000, ready_observed_monotonic_ns=t-100,
                read_complete_monotonic_ns=t, unix_time_ns=time.time_ns(), raw_counts=raw, max_clock_high_bound_us=9.)


class FakeClient:
    def __init__(self, delay=0):
        self.vector=be.STOP
        self.sent=[]
        self.delay=delay
        self.battery_state=0
        self.armed=False
        self.lose_ack=False
        self.lose_stop=False

    def query(self, n):
        time.sleep(self.delay)
        if n==101:
            return struct.pack("<HHHIB",250,0,33,int(self.armed),0)
        if n==119:
            return b"\0"
        if n==130:
            return struct.pack("<BHBHhBH",2,0,80,0,40,self.battery_state,800)
        if n==104:
            return struct.pack("<8H",*self.vector,0,0,0,0)
        if n==139:
            return b"\4"+b"".join(struct.pack("<IHBHHH",0 if v==1000 else 15000,0,0,0,0,0) for v in self.vector)
        raise AssertionError(n)

    def _exchange(self, packet, number):
        assert number==214
        self.vector=struct.unpack("<4H",packet[5:-1])
        self.sent.append(self.vector)
        if self.vector!=be.STOP and self.lose_ack:
            raise TimeoutError("accepted command but reply lost")
        if self.vector==be.STOP and self.lose_stop:
            raise TimeoutError("stop reply lost")
        return b""


class FakeSensor:
    def __init__(self):
        self.closed=False
    def read(self):
        time.sleep(.087)
        return sample()
    def close(self):
        self.closed=True


class Tests(unittest.TestCase):
    @contextlib.contextmanager
    def bench(self,preset="repeatability",client=None,reader=None):
        with tempfile.TemporaryDirectory() as tmp:
            log=be.Log(tmp)
            args=be.parser(preset).parse_args([])
            client=client or FakeClient()
            reader=reader or SimpleNamespace(samples=queue.Queue(),error=None)
            b=be.Bench(args,log,reader,CAL,client,PROFILE,[0])
            b.force_time=be.mono()
            b.latest_force=b.baseline=1.
            try:
                yield b,client,Path(tmp)
            finally:
                log.close()

    def test_preview_opens_no_devices(self):
        with patch.object(be,"run",side_effect=AssertionError("must not access hardware")),contextlib.redirect_stdout(io.StringIO()):
            for preset in be.PRESETS:
                self.assertEqual(be.main([],preset=preset),0)

    def test_trajectory_ceilings_cycles_and_holdout(self):
        repeat=be.plan_for("repeatability")
        self.assertEqual(sum(p["name"].startswith("up_") for p in repeat),4)
        self.assertEqual([p["role"] for p in repeat[-2:]],["validation"]*2)
        for preset in be.PRESETS:
            for p in be.plan_for(preset):
                self.assertTrue(all(1000<=v<=1450 for v in p["commands"]))
        for p in be.plan_for("individual"):
            self.assertLessEqual(sum(v>1000 for v in p["commands"]),1)
        self.assertTrue(all(p["commands"][0]==1000 for p in be.plan_for("individual",motor=3)))

    def test_explicit_execution_declarations_required(self):
        cases=[("repeatability",["--execute"]),
               ("repeatability",["--execute","--bench-ready"]),
               ("individual",["--execute","--bench-ready","--battery-id","B1"]),
               ("weights",["--execute"])]
        with patch.object(be,"run",side_effect=AssertionError("hardware must not be accessed")),contextlib.redirect_stderr(io.StringIO()),contextlib.redirect_stdout(io.StringIO()):
            for preset,args in cases:
                with self.assertRaises(SystemExit):
                    be.main(args,preset=preset)

    def test_force_limit_logs_sample_then_prevents_motor_write(self):
        with self.bench() as (b,c,_):
            b.reader.samples.put(sample(4000))
            with self.assertRaisesRegex(RuntimeError,"cutoff"):
                b.send([1250]*4,"test")
            self.assertEqual(c.sent,[])
            self.assertEqual(b.log.counts["force"],1)

    def test_stale_force_worker_error_and_rail_prevent_motor_write(self):
        for failure in ("stale","worker","rail"):
            with self.bench() as (b,c,_):
                if failure=="stale": b.force_time-=1_000_000_000
                elif failure=="worker": b.reader.error="clock pulse too long"
                else: b.reader.samples.put(sample(8388607))
                with self.assertRaises(RuntimeError): b.send([1250]*4,"test")
                self.assertEqual(c.sent,[])

    def test_battery_warning_or_armed_prevents_motor_write(self):
        for field in ("battery_state","armed"):
            with self.bench() as (b,c,_):
                setattr(c,field,1)
                with self.assertRaises(RuntimeError): b.send([1250]*4,"test")
                self.assertEqual(c.sent,[])

    def test_read_only_modes_cannot_send_motor_packets(self):
        for preset in ("weights","probe"):
            with self.bench(preset=preset) as (b,c,_):
                with self.assertRaises(RuntimeError): b.send(be.STOP,"test")
                self.assertEqual(c.sent,[])

    def test_rapid_rpm_sampling_does_not_wait_for_force(self):
        sensor=FakeSensor()
        reader=be.ForceReader(lambda:sensor)
        with self.bench(preset="probe",client=FakeClient(delay=.002),reader=reader) as (b,c,path),contextlib.redirect_stdout(io.StringIO()):
            try:
                b.initialize_acquisition()
                b.hold(dict(name="probe",role="read_only",duration_s=.8))
            finally:
                reader.cancel.set(); reader.join(1.)
            self.assertTrue(sensor.closed)
            self.assertEqual(c.sent,[])
            with open(path/"rpm.csv") as f:
                rr=list(csv.DictReader(f))
            with open(path/"force.csv") as f:
                ff=list(csv.DictReader(f))
            self.assertGreater(len(rr),3*len(ff))
            gaps=[(int(y["reply_ns"])-int(x["reply_ns"]))/1e9 for x,y in zip(rr,rr[1:])]
            self.assertLess(statistics.median(gaps),.04)
            self.assertTrue(all(int(r["reply_ns"])>=int(r["request_start_ns"]) for r in rr))

    def test_zero_rpm_after_startup_is_not_accepted(self):
        with self.bench() as (b,c,_):
            b.vector=(1250,)*4
            b.command_ack_ns=be.mono()-1_100_000_000
            with self.assertRaisesRegex(RuntimeError,"No reported RPM"):
                b.poll_rpm()
            self.assertEqual(b.log.counts["rpm"],1)

    def run_fault(self,fault,lose_ack=False,lose_stop=False,log_failure=False):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/"cal.json").write_text(json.dumps(CAL))
            (root/"profile.json").write_text(json.dumps(PROFILE))
            args=be.parser("repeatability").parse_args(["--execute","--bench-ready","--battery-id","FAKE",
                "--calibration",str(root/"cal.json"),"--profile",str(root/"profile.json"),"--output",str(root/"run")])
            c=FakeClient()
            c.lose_ack,c.lose_stop=lose_ack,lose_stop
            fd=os.open(root/"fake_serial",os.O_RDWR|os.O_CREAT,0o600)
            transport=SimpleNamespace(fd=fd,close=lambda:os.close(fd))
            def initialize(b):
                b.force_time=be.mono(); b.latest_force=b.baseline=1.
            def run_plan(b,plan):
                b.send([1250]*4,"injected")
                if fault is not None: raise fault
            original_row=be.Log.row
            def row(log,name,data):
                if log_failure and name=="commands": raise OSError("disk unavailable after command")
                return original_row(log,name,data)
            with patch.object(be,"USBSerial",return_value=transport),patch.object(be,"MSP",return_value=c),\
                 patch.object(be,"inspect_controller",return_value=PROFILE),\
                 patch.object(be.Bench,"initialize_acquisition",initialize),patch.object(be.Bench,"run_plan",run_plan),\
                 patch.object(be.Log,"row",row),contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                rc=be.run(args,[])
            summary=json.loads((root/"run/summary.json").read_text())
            return rc,c.sent,summary

    def test_ctrl_c_triggers_three_stops_in_real_finally(self):
        rc,packets,s=self.run_fault(KeyboardInterrupt())
        self.assertEqual(rc,130)
        self.assertEqual(packets,[(1250,)*4]+[be.STOP]*3)
        self.assertEqual(s["stop_acknowledgements"],3)

    def test_runtime_error_and_normal_completion_both_stop(self):
        for fault in (RuntimeError("sensor failed"),None):
            rc,packets,s=self.run_fault(fault)
            self.assertEqual(rc,1 if fault else 0)
            self.assertEqual(packets[-3:],[be.STOP]*3)

    def test_lost_ack_still_sends_stop(self):
        rc,packets,s=self.run_fault(None,lose_ack=True)
        self.assertEqual(rc,1)
        self.assertEqual(packets,[(1250,)*4]+[be.STOP]*3)

    def test_data_logging_failure_cannot_prevent_stop(self):
        rc,packets,s=self.run_fault(None,log_failure=True)
        self.assertEqual(rc,1)
        self.assertEqual(packets[-3:],[be.STOP]*3)

    def test_unacknowledged_stop_is_not_reported_success(self):
        rc,packets,s=self.run_fault(None,lose_stop=True)
        self.assertEqual(rc,1)
        self.assertEqual(s["status"],"stop_not_acknowledged")
        self.assertFalse(s["cleanup_stop_acknowledged"])


if __name__=="__main__":
    unittest.main()
