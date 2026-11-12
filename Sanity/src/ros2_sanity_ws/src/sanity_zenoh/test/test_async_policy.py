import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from sanity_zenoh.async_policy import AsyncPolicy


class AsyncHistoryTests(unittest.TestCase):
    def test_slow_inference_does_not_block_ticks_and_commits_every_selected_sample(self):
        gate = threading.Event()
        class Actor:
            def __init__(self,*args,**kwargs):
                self.info = {}
            def call(self,op,deadline,**request):
                if op == "infer":
                    gate.wait(.5)
                    return {"action":[999.]*4}
                return {"action":request["ctbr"]}
            def close(self):
                gate.set()
        with tempfile.TemporaryDirectory() as directory,patch("sanity_zenoh.async_policy.ActorProcess",Actor):
            worker = AsyncPolicy(None,None,None,Path(directory))
            try:
                start = time.monotonic()
                for tick in range(6):
                    worker.observe({"tick":tick,"state":[],"goal":[],"state_stamp_ns":tick})
                    worker.commit({"tick":tick,"selected_ctbr":[tick*.01,0.,0.,0.]})
                self.assertLess(time.monotonic()-start,.02)
                self.assertEqual(worker.processed,0)
                with self.assertRaises(ValueError):
                    worker.observe({"tick":8})
                gate.set()
                deadline = time.monotonic()+1
                while worker.processed<6 and time.monotonic()<deadline:
                    time.sleep(.001)
                self.assertEqual(worker.processed,6)
                self.assertEqual(worker.latest()["tick"],5)
                rows = [json.loads(line) for line in (Path(directory)/"history.jsonl").read_text().splitlines()]
                self.assertEqual([r["observation"]["tick"] for r in rows],list(range(6)))
                self.assertTrue(all(r["committed"]["action"]==r["selection"]["selected_ctbr"] for r in rows))
            finally:
                worker.close()

    def test_next_inference_waits_for_selected_history_not_shadow_prediction(self):
        calls = []
        class Actor:
            def __init__(self,*args,**kwargs):
                self.info = {}
            def call(self,op,deadline,**request):
                calls.append(op)
                return {"action":[99.]*4} if op=="infer" else {"action":request["ctbr"]}
            def close(self):
                pass
        with tempfile.TemporaryDirectory() as directory,patch("sanity_zenoh.async_policy.ActorProcess",Actor):
            worker = AsyncPolicy(None,None,None,Path(directory))
            try:
                for tick in range(2):
                    worker.observe({"tick":tick,"state":[],"goal":[],"state_stamp_ns":tick})
                until = time.monotonic()+1
                while worker.latest() is None and time.monotonic()<until:
                    time.sleep(.001)
                self.assertEqual(worker.latest()["tick"],0)
                time.sleep(.01)
                self.assertEqual(calls,["infer"])
                worker.commit({"tick":0,"selected_ctbr":[.1,0.,0.,0.]})
                until = time.monotonic()+1
                while worker.latest()["tick"]<1 and time.monotonic()<until:
                    time.sleep(.001)
                self.assertEqual(calls,["infer","commit","infer"])
                worker.commit({"tick":1,"selected_ctbr":[.2,0.,0.,0.]})
            finally:
                worker.close()
            self.assertEqual(calls,["infer","commit","infer","commit"])

    def test_worker_failure_is_reported_and_never_resets_history(self):
        class Actor:
            def __init__(self,*args,**kwargs):
                self.info = {}
            def call(self,*args,**kwargs):
                raise RuntimeError("injected worker crash")
            def close(self):
                pass
        with tempfile.TemporaryDirectory() as directory,patch("sanity_zenoh.async_policy.ActorProcess",Actor):
            worker = AsyncPolicy(None,None,None,Path(directory))
            try:
                worker.observe({"tick":0,"state":[],"goal":[],"state_stamp_ns":0})
                deadline = time.monotonic()+1
                while not worker.error and time.monotonic()<deadline:
                    time.sleep(.001)
                self.assertIn("injected worker crash",worker.error)
                with self.assertRaises(RuntimeError):
                    worker.observe({"tick":1})
                self.assertEqual(worker.observed,1)
                self.assertEqual(worker.processed,0)
            finally:
                worker.close()


if __name__ == "__main__":
    unittest.main()
