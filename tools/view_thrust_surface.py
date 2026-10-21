#!/usr/bin/env python3
"""Rotate, pan and zoom the shared direct-command / FC-voltage thrust surface.

Run from any directory; the curated parameter file is the default model.
The local fullrange archive supplies measured points when it is available.
Only NumPy, SciPy and Matplotlib (already in requirements.txt) are needed.

  python tools/view_thrust_surface.py
  python tools/view_thrust_surface.py --save build/thrust_surface.png --no-show

Controls: left-drag rotate; middle-drag pan; right-drag zoom. The toolbar's
Pan tool also supports left-drag panning. Press R or click Reset view to reset.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import Delaunay

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT.parents[1] / 'outputs/sanity_fullrange_id_20261020'
MODEL = ROOT / 'parameters/identified_actuators.json'
MEASUREMENTS = ARCHIVE / 'thrust_command_voltage_surface.json'
COLORS = ['#28699c', '#dc8d21', '#25917c', '#9a629f']


def read_model(path):
    """Accept either the curated hardware JSON or the full fitted-surface JSON."""
    data = json.loads(path.read_text())
    if 'models' in data:
        model = data['models']['direct_motor_thrust']
        return dict(
            coefficients=model['coefficients_N'],
            powers=model['coefficient_powers_q_z'],
            normalization=model['normalization'],
            hull=model['domain']['convex_hull_command_voltage_V'],
        )
    return dict(
        coefficients=data['coefficients'],
        powers=data['coefficient_powers_q_z'],
        normalization=dict(command_center=1550., command_scale=450.,
                           voltage_center_V=7.5, voltage_scale_V=.5),
        hull=data['domain']['convex_hull_command_voltage_V'],
    )


def predict(model, command, voltage):
    n = model['normalization']
    q = (np.asarray(command) - n['command_center']) / n['command_scale']
    z = (np.asarray(voltage) - n['voltage_center_V']) / n['voltage_scale_V']
    return sum(c * q**i * z**j
               for c, (i, j) in zip(model['coefficients'], model['powers']))


def supported_mesh(model, resolution):
    hull = np.asarray(model['hull'])
    command, voltage = np.meshgrid(
        np.linspace(hull[:, 0].min(), hull[:, 0].max(), resolution),
        np.linspace(hull[:, 1].min(), hull[:, 1].max(), resolution),
    )
    # Fit support is a joint convex hull, not the enclosing rectangle.
    supported = Delaunay(hull).find_simplex(
        np.c_[command.ravel(), voltage.ravel()], tol=1e-10
    ).reshape(command.shape) >= 0
    thrust = np.where(supported, predict(model, command, voltage), np.nan)
    return command, voltage, thrust


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', type=Path, default=MODEL)
    parser.add_argument('--measurements', type=Path,
                        help='Full thrust_command_voltage_surface.json containing points; auto-detected locally.')
    parser.add_argument('--no-measurements', action='store_true')
    parser.add_argument('--resolution', type=int, default=65, help='Grid points per axis (default: 65).')
    parser.add_argument('--save', type=Path, help='Also save a static PNG/PDF/SVG preview.')
    parser.add_argument('--no-show', action='store_true', help='Headless export; requires --save.')
    args = parser.parse_args()
    if not 12 <= args.resolution <= 250:
        parser.error('--resolution must be between 12 and 250')
    if args.no_show and not args.save:
        parser.error('--no-show requires --save')
    if not args.model.is_file():
        parser.error(f'Model not found: {args.model}')
    points_path = args.measurements or (MEASUREMENTS if MEASUREMENTS.is_file() else None)
    if args.measurements and not args.measurements.is_file():
        parser.error(f'Measurements not found: {args.measurements}')

    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.widgets import Button, CheckButtons, Slider

    for style in ['Regular', 'Heavy']:
        font = Path.home() / f'.local/share/fonts/Avenir {style}.ttf'
        if font.exists():
            font_manager.fontManager.addfont(str(font))
    plt.rcParams.update({'font.family': ['Avenir', 'DejaVu Sans'], 'font.size': 10})
    model = read_model(args.model)
    command, voltage, thrust = supported_mesh(model, args.resolution)
    points = []
    if not args.no_measurements and points_path:
        points = json.loads(points_path.read_text())['points']
        xy = np.array([[p['command'], p['FC_voltage_V']] for p in points])
        if np.any(Delaunay(model['hull']).find_simplex(xy, tol=1e-10) < 0):
            parser.error('Measurement points extend beyond this model’s recorded joint support.')

    fig = plt.figure(figsize=(12, 8.5))
    fig.canvas.manager.set_window_title('Sanity | interactive thrust surface')
    ax = fig.add_axes([.06, .19, .79, .70], projection='3d')
    surface = ax.plot_surface(command, voltage, thrust, cmap='viridis', alpha=.65,
                              rcount=args.resolution, ccount=args.resolution,
                              linewidth=0, antialiased=True)
    scatters = []
    for motor in range(1, 5):
        rows = [p for p in points if p['motor'] == motor]
        if rows:
            scatters.append(ax.scatter(
                [p['command'] for p in rows], [p['FC_voltage_V'] for p in rows],
                [p['thrust_N'] for p in rows], color=COLORS[motor-1], s=16,
                alpha=.55, depthshade=False, edgecolors='none', label=f'M{motor}',
            ))
    if scatters:
        ax.legend(loc='upper left', title='Measured motors', fontsize=9)
    ax.set(xlabel='Direct MSP motor command [count]',
           ylabel='Delivered FC bus voltage [V]', zlabel='Per-rotor thrust [N]')
    ax.set_box_aspect((1.5, 1., 1.))
    ax.xaxis.labelpad = 12
    ax.yaxis.labelpad = 12
    ax.zaxis.labelpad = 10
    ax.view_init(elev=25, azim=-125, roll=0)
    ax.mouse_init(rotate_btn=1, pan_btn=2, zoom_btn=3)
    limits = (ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d())
    fig.suptitle('Sanity: shared thrust–motor-command–voltage model',
                 fontsize=16, fontweight='heavy', y=.965)
    fig.text(.5, .918, f'One rotor · fitted surface + {len(points)} measured points · measured joint support only',
             ha='center', color='#526273')
    fig.text(.5, .048,
             'Left-drag: rotate   |   Middle-drag: pan   |   Right-drag: zoom   |   R: reset\n'
             'For a trackpad, select Pan in the toolbar, then left-drag. Deselect it to rotate.',
             ha='center', fontsize=10, linespacing=1.6)
    fig.text(.5, .012,
             'Direct motor-test command; output is thrust per rotor. This is separate from the CTBR RC-throttle model.',
             ha='center', fontsize=9, color='#526273')

    opacity_ax = fig.add_axes([.24, .12, .36, .025])
    opacity = Slider(opacity_ax, 'Surface opacity', .1, 1., valinit=.65, valfmt='%.2f')
    def change_opacity(value):
        surface.set_alpha(value)
        fig.canvas.draw_idle()
    opacity.on_changed(change_opacity)
    checks_ax = fig.add_axes([.82, .40, .17, .10])
    checks = CheckButtons(checks_ax, ['Measurements'], [bool(scatters)])
    def toggle_points(_):
        for scatter in scatters:
            scatter.set_visible(checks.get_status()[0])
        fig.canvas.draw_idle()
    checks.on_clicked(toggle_points)
    reset_ax = fig.add_axes([.82, .29, .14, .05])
    reset_button = Button(reset_ax, 'Reset view')
    def reset_view(_=None):
        ax.set_xlim3d(limits[0])
        ax.set_ylim3d(limits[1])
        ax.set_zlim3d(limits[2])
        ax.view_init(elev=25, azim=-125, roll=0)
        fig.canvas.draw_idle()
    reset_button.on_clicked(reset_view)
    fig.canvas.mpl_connect('key_press_event', lambda event: reset_view() if event.key == 'r' else None)
    # Keep widget callbacks alive for the lifetime of the figure, including nonblocking use.
    fig._sanity_controls = (opacity, checks, reset_button)
    print(f'Model: {args.model.resolve()}')
    print(f'Measured points: {len(points)}')
    print('Left-drag rotate; middle-drag pan; right-drag zoom. Press R to reset.')
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=200)
        print(f'Preview: {args.save.resolve()}')
    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == '__main__':
    main()
