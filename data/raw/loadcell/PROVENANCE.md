# Weight-sweep calibration provenance

- Source: `unitree@192.168.123.15:~/sanity_thrust_test/loadcell_calibration/`, copied to this package on 2026-10-13. The original Jetson files were not modified.
- Calibration ID: `c1bd33fe-d4b3-4c22-bb7b-db5bdfab119c`.
- Hardware recorded by the reader: P3768 carrier with Orin NX module, HX711 channel A, gain 128, physical DOUT pin 29 and SCK pin 31.
- Operator-entered sequence: nominal 100 g increments from 0 to 1000 g and back to zero; 40 accepted samples per point. The session contains 21 points and 840 samples.
- Fit: unweighted linear regression of ascending load means; descending points are used to assess return behavior. See `report.txt` and `calibration.json` for coefficients and residuals.
- The load-cell capacity is unverified. Reference-weight tolerances, load path geometry and absolute force accuracy were not independently verified. Use only within the measured 0–1000 g range and matching loading direction/setup pending those checks.
- `readings.json` is the preserved raw calibration record. `calibration_plot.png` was generated locally from it with `plot_loadcell_calibration.py`; the plot is a derived artifact.
