#!/usr/bin/env python3
"""Plot a completed calibration using matplotlib; run on any machine with matplotlib."""
import argparse
import json
from pathlib import Path

from calibrate_loadcell import fit_session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("readings", type=Path)
    parser.add_argument("--output", type=Path, help="default: calibration_plot.png alongside input")
    args = parser.parse_args()
    output = args.output or args.readings.with_name("calibration_plot.png")
    if output.exists():
        parser.error("Output already exists; choose another path")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        parser.error("matplotlib is needed for plots only; collection/fitting use the standard library")
    session = json.loads(args.readings.read_text())
    fit = fit_session(session)
    figure, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    for sweep, marker in [("ascending", "o"), ("descending", "s")]:
        points = [p for p in fit["points"] if p["sweep"] == sweep]
        if not points:
            continue
        x = [p["mass_g"] for p in points]
        axes[0].errorbar(x, [p["mean_counts"] for p in points],
                         yerr=[p["std_counts"] for p in points], fmt=marker, label=sweep + " mean ± sample SD")
        axes[1].plot(x, [p["residual_equivalent_g"] for p in points], marker, label=sweep)
    x = fit["reference_mass_range_g"]
    axes[0].plot(x, [fit["fit_intercept_counts"] + fit["gradient_counts_per_g"] * a for a in x],
                 "k-", label="ascending linear fit")
    axes[0].set_ylabel("ADC counts")
    axes[1].set_ylabel("Residual (g equivalent)")
    axes[1].set_xlabel("Total added reference mass (g)")
    axes[1].axhline(0, color="k", linewidth=0.7)
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend()
    figure.suptitle("Load-cell calibration: %.6g counts/g, R² = %.7f" %
                    (fit["gradient_counts_per_g"], fit["r_squared"]))
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)
    print(output)


if __name__ == "__main__":
    main()
