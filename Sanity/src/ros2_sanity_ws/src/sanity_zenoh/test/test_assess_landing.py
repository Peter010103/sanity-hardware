from sanity_zenoh.standalone_paths import load_json
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class AssessmentTests(unittest.TestCase):
    def test_missing_circle_and_single_circle_sample_produce_failed_reports(self):
        assessor = Path(__file__).resolve().parents[3]/"scripts/assess_landing.py"
        for phase in ("circle_start_hold", "circle_cruise"):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as directory:
                run = Path(directory)
                (run/"controller").mkdir()
                receipt = {"recording_error":None,"history_processed":0,"history_submitted":0,
                    "policy":{"hover_ctrl":[.65,0,0,0],"scaling":[.2,1,1,1]},
                    "landing_start_s":60.,"landing_touchdown_s":None,"landed":False,
                    "landing_reason":"mission_complete","ground_root":.028,
                    "parameters":{"enable_output":True,"origin_xy":[0.,0.]},
                    "fallback":False,"error":"PD: ValueError: Landing timeout: touchdown could not be confirmed",
                    "policy_start_s":None,"orbit_start_s":None,"mapping":{}}
                control = {"tick":0,"time_s":84.,"state":[0.,0.,.052,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.],
                    "reference":{"position":[.5,0.,1.],"phase":phase},"command_limited":False,
                    "command":[.63,0.,0.,0.],"history_event":{},"history_observation":{},"policy_weight":0.}
                (run/"controller/receipt.json").write_text(json.dumps(receipt))
                (run/"controller/control.jsonl").write_text(json.dumps(control)+"\n")
                (run/"experiment_receipt.json").write_text(json.dumps({"rosbag_returncode":0}))
                result = subprocess.run([sys.executable,str(assessor),str(run)],capture_output=True,text=True)
                self.assertEqual(result.returncode,1,result.stderr)
                self.assertNotIn("Traceback",result.stderr)
                summary = load_json(run / 'summary.json')
                self.assertFalse(summary["passed"])
                self.assertFalse(summary["checks"]["two_circle_laps"])
                self.assertIsNone(summary["metrics"]["measured_laps"])
                self.assertEqual(summary["controller_error"],receipt["error"])
                self.assertAlmostEqual(summary["metrics"]["final_height_above_initial_ground_m"],.024)
                self.assertTrue((run/"experiment.pdf").is_file())
