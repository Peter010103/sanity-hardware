#!/usr/bin/env python3
"""Plot collective thrust and measured motor-speed steps for the README."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from fit_actuators import load_runs, extract_edges

ROOT = Path(__file__).resolve().parents[1]
report = json.loads((ROOT / 'data/processed/actuator/fit_report.json').read_text())
parameters = json.loads((ROOT / 'parameters/actuators.json').read_text())
k = parameters['thrust']['k_N_per_rad_s_squared']
plt.style.use('default')
plt.rcParams.update({'lines.linewidth': .8, 'grid.linestyle': ':'})

# Collective thrust uses the RMS of the four individual angular speeds.
with (ROOT / 'data/processed/actuator/static_points.csv').open() as f:
    points = list(csv.DictReader(f))
errors = [k * sum(float(p[f'omega_squared_M{i}']) for i in range(1, 5))
          - float(p['thrust_N']) for p in points]
rmse = float(np.sqrt(np.mean(np.square(errors))))
assert abs(rmse - report['models']['extended_static_thrust']['all_data_fit']['rmse']) < 1e-12
collective = [p for p in points if p['context'] == 'collective']
omega = np.array([np.sqrt(sum(float(p[f'omega_squared_M{i}']) for i in range(1, 5)) / 4)
                  for p in collective])
thrust = np.array([float(p['thrust_N']) for p in collective])

fig, ax = plt.subplots(figsize=(7, 4.5), layout='constrained')
ax.plot(omega, thrust, 'o', ms=4, color='tab:blue', label='Measurements')
speed = np.linspace(omega.min(), omega.max(), 400)
ax.plot(speed, 4 * k * speed**2, color='tab:orange', lw=.8,
        label=r'Fit: $T = 4k_T\omega_{\mathrm{RMS}}^2$')
ax.set(title='Collective thrust', xlabel=r'RMS rotor speed, $\omega_{\mathrm{RMS}}$ (rad/s)',
       ylabel='Thrust (N)', xlim=(0, 5500), ylim=(0, 3.6))
ax.xaxis.set_major_locator(MultipleLocator(1000))
ax.yaxis.set_major_locator(MultipleLocator(0.5))
ax.grid(alpha=.25)
ax.legend()
fig.savefig(ROOT / 'media/thrust-fit.png', dpi=180)
plt.close(fig)

# Use the same timestamped edges and settled endpoint normalization as the fit.
# These compare response shapes, not command-to-speed prediction accuracy.
runs, _ = load_runs(ROOT / 'data/raw/experiments')
edges, _, _ = extract_edges(runs)
delay = parameters['response']['delay_s']
tau = parameters['response']['tau_s']
time = np.linspace(-.05, .30, 500)
rise = 1 - np.exp(-np.maximum(0, time - delay) / tau)
fig, axes = plt.subplots(1, 2, figsize=(9, 3.5), sharex=True, sharey=True, layout='constrained')
counts = {}
for ax, direction, title in zip(axes, ['up', 'down'], ['Speed increase', 'Speed decrease']):
    group = [e for e in edges if e['direction'] == direction]
    counts[direction] = len(group)
    for i, edge in enumerate(group):
        visible = (edge['t'] >= time[0]) & (edge['t'] <= time[-1])
        measured = edge['z'] if direction == 'up' else 1 - edge['z']
        ax.plot(edge['t'][visible] * 1000, measured[visible], '.', ms=3,
                color='tab:blue', alpha=.2, label='Measurements' if i == 0 else None)
    prediction = rise if direction == 'up' else 1 - rise
    ax.plot(time * 1000, prediction, color='tab:orange', lw=.8, label='First-order model')
    ax.axvline(0, color='0.5', ls='--', lw=.8)
    ax.set(title=title, xlabel='Time from command step (ms)', xlim=(-50, 300), ylim=(-.2, 1.2))
    ax.xaxis.set_major_locator(MultipleLocator(50))
    ax.set_yticks([0, .25, .5, .75, 1])
    ax.grid(alpha=.25)
    ax.legend(loc='lower right' if direction == 'up' else 'upper right', fontsize=9)
axes[0].set_ylabel('Normalised rotor speed')
fig.savefig(ROOT / 'media/response-fit.png', dpi=180)
plt.close(fig)
print(json.dumps(dict(collective_observations=len(collective), all_51_fit_RMSE_N=rmse,
                      speed_step_traces=counts, delay_ms=delay*1000, time_constant_ms=tau*1000,
                      step_normalization='measured settled endpoints; 0 low speed, 1 high speed')))
