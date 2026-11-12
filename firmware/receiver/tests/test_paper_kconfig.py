"""Resolve the actual project Kconfig with the ESP-IDF parser, without a build."""
from pathlib import Path
import unittest
try:
    from esp_kconfiglib import Kconfig
except ImportError:
    Kconfig=None

@unittest.skipUnless(Kconfig,'Run with ESP-IDF Python for real configuration resolution')
class PaperKconfigTests(unittest.TestCase):
    def test_paper_native_hides_legacy_hover_choice_even_from_old_selection(self):
        path=Path(__file__).resolve().parents[1]/'main/Kconfig.projbuild'
        for previous in ('SANITY_ESTIMATOR_PID_HOVER','SANITY_ESTIMATOR_INDI_HOVER'):
            k=Kconfig(str(path),warn=False)
            k.syms['SANITY_MSP_UNIFIED'].set_value(2)
            k.syms['SANITY_MOCAP_STREAM'].set_value(2)
            k.syms[previous].set_value(2)
            self.assertEqual(k.syms[previous].str_value,'y')
            k.syms['SANITY_PAPER_NATIVE'].set_value(2)
            k.syms['SANITY_ESTIMATOR_EXPERIMENT'].set_value(2)
            self.assertEqual(k.syms['SANITY_PAPER_NATIVE'].str_value,'y')
            self.assertEqual(k.syms['SANITY_ESTIMATOR_EXPERIMENT'].str_value,'y')
            self.assertEqual(k.syms['SANITY_ESTIMATOR_PID_HOVER'].str_value,'n')
            self.assertEqual(k.syms['SANITY_ESTIMATOR_INDI_HOVER'].str_value,'n')

if __name__=='__main__':unittest.main(verbosity=2)
