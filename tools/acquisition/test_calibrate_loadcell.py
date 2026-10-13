"""Synthetic fitting checks, never a calibration of the real load cell."""
import json
from pathlib import Path
import tempfile
import unittest

import calibrate_loadcell as cal
import hx711_jetson as hx


def session(gradient=2.5, descending_offset=0, sign=1):
    points = []
    for sweep, mass in cal.schedule(1000, 100):
        mean = 200000 + gradient * mass + (descending_offset if sweep == "descending" else 0)
        points.append({"sweep": sweep, "mass_g": mass,
                       "samples": [{"raw_counts": mean + noise} for noise in (-1, 0, 1)]})
    return {"collection_status": "complete", "points": points, "gravity_m_s2": hx.G0,
            "force_sign": sign, "hardware": {"backend": "synthetic_test_only"},
            "load_cell_id": "synthetic", "rig_id": "synthetic", "positive_direction": "test axis"}


class Tests(unittest.TestCase):
    def test_gradient_and_units(self):
        fit = cal.fit_session(session())
        self.assertAlmostEqual(fit["gradient_counts_per_g"], 2.5)
        self.assertAlmostEqual(fit["counts_per_N"], 2500 / hx.G0)
        self.assertAlmostEqual(fit["r_squared"], 1)
        self.assertAlmostEqual(fit["ascending_residual_RMSE_equivalent_g"], 0)

    def test_descending_is_validation_not_fit_and_zero_is_retared(self):
        data = session(descending_offset=20)
        fit = cal.fit_session(data)
        self.assertAlmostEqual(fit["gradient_counts_per_g"], 2.5)
        self.assertAlmostEqual(fit["max_abs_hysteresis_equivalent_g"], 8)
        self.assertAlmostEqual(fit["zero_return_drift_equivalent_g"], 8)
        self.assertEqual(fit["final_zero_counts"], 200020)

    def test_polarity_and_force_sign(self):
        for gradient in (-5, 5):
            for sign in (-1, 1):
                data = session(gradient=gradient, sign=sign)
                fit = cal.fit_session(data)
                document = cal.calibration_document(data, fit)
                self.assertAlmostEqual(hx.force_from_counts(200000 + gradient * 1000, document), sign * hx.G0)

    def test_compatible_calibration_file(self):
        data = session()
        document = cal.calibration_document(data, cal.fit_session(data))
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory)/"calibration.json"
            file.write_text(json.dumps(document))
            loaded = hx.load_calibration(file)
            self.assertAlmostEqual(hx.force_from_counts(loaded["zero_counts"] + loaded["counts_per_N"], loaded), 1)
            self.assertEqual(loaded["tare_history"], [])

    def test_incomplete_saturated_and_no_response_are_rejected(self):
        data = session()
        data["collection_status"] = "interrupted"
        with self.assertRaises(ValueError): cal.fit_session(data)
        data = session()
        data["points"][-1]["samples"][0]["raw_counts"] = hx.RAILS[0]
        with self.assertRaises(ValueError): cal.fit_session(data)
        with self.assertRaises(ValueError): cal.fit_session(session(gradient=0))

    def test_schedule_and_total_mass_convention(self):
        steps = cal.schedule(1000, 100)
        self.assertEqual(steps[:3], [("ascending", 0), ("ascending", 100), ("ascending", 200)])
        self.assertEqual(steps[-2:], [("descending", 100), ("descending", 0)])
        self.assertEqual(len(steps), 21)
        with self.assertRaises(ValueError): cal.schedule(950, 100)


if __name__ == "__main__":
    unittest.main()
