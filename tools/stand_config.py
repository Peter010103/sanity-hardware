"""Local paths and device selection; importing this module opens no devices."""
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
ROOT = Path(os.environ.get("SANITY_RUN_ROOT", str(REPO / "runs"))).expanduser().resolve()
FC_PORT = os.environ.get("SANITY_FC_PORT", "/dev/serial/by-id/usb-Betaflight_Betaflight_STM32F405_0x8000000-if00")
PSU_PORT = os.environ.get("SANITY_PSU_PORT", "/dev/serial/by-id/usb-Nuvoton_KORAD_USB_Mode_00002B240000-if00")
FC_UID = os.environ.get("SANITY_FC_UID", "")
FC_PROFILE = Path(os.environ.get("SANITY_FC_PROFILE", str(REPO / "config/fc_profile.json"))).expanduser().resolve()
CALIBRATION = Path(os.environ.get("SANITY_CALIBRATION", str(REPO / "config/loadcell_calibration.json"))).expanduser().resolve()
SLOW_LIBRARY = HERE / "libhx711_slow10.so"
# Separate GPL dependency, pinned and patched by scripts/setup-psu.sh.
sys.path.insert(0, str(REPO / "deps/tenma-serial"))
