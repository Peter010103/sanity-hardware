#!/usr/bin/env python3
"""Reproduce the weight calibration and export preserved samples; no hardware I/O."""
import csv
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/acquisition'))
from calibrate_loadcell import fit_session

def csv_write(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

def main():
    src = ROOT / 'data/raw/loadcell'
    out = ROOT / 'data/processed/loadcell'
    out.mkdir(exist_ok=True)
    raw = json.loads((src / 'readings.json').read_text())
    recorded = json.loads((src / 'calibration.json').read_text())
    fit = fit_session(raw)
    assert abs(fit['counts_per_N'] - recorded['counts_per_N']) < 1e-8
    assert fit['reference_mass_range_g'] == [0,1000]
    samples = [dict(sweep=p['sweep'], mass_g=p['mass_g'], sample_in_level=i, **sample)
               for p in raw['points'] for i, sample in enumerate(p['samples'])]
    assert len(samples) == 840
    csv_write(out / 'loadcell_samples.csv', samples)
    csv_write(out / 'loadcell_level_means.csv', fit['points'])
    (out / 'loadcell_fit.json').write_text(json.dumps(fit, indent=2) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'lines.linewidth': .8, 'grid.linestyle': ':'})
    fig, ax = plt.subplots(2, 1, figsize=(8,7), sharex=True)
    for name, marker in [('ascending','o'),('descending','s')]:
        points = [p for p in fit['points'] if p['sweep'] == name]
        ax[0].errorbar([p['mass_g'] for p in points], [p['mean_counts'] for p in points],
                      yerr=[p['std_counts'] for p in points], fmt=marker, label=name+' mean ± sample SD')
        ax[1].plot([p['mass_g'] for p in points], [p['residual_equivalent_g'] for p in points], marker, label=name)
    xx = fit['reference_mass_range_g']
    ax[0].plot(xx, [fit['fit_intercept_counts'] + fit['gradient_counts_per_g']*x for x in xx], 'k-', label='ascending linear fit')
    ax[0].set_ylabel('ADC counts'); ax[1].set_ylabel('Residual (g equivalent)')
    ax[1].set_xlabel('Added nominal reference mass (g)'); ax[1].axhline(0,color='k',lw=.8)
    for a in ax: a.grid(alpha=.2); a.legend()
    fig.suptitle('Sanity HX711 calibration: residuals exclude systematic uncertainty')
    fig.tight_layout(); fig.savefig(ROOT / 'media/loadcell-fit.png',dpi=160); plt.close(fig)
    print(json.dumps(dict(samples=len(samples), levels=len(fit['points']),
                         calibration_gradient_matches=True, counts_per_N=fit['counts_per_N'])))

if __name__ == '__main__':
    main()
