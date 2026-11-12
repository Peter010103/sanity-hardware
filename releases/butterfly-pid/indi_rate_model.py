"""Nominal INDI command-response model and causal selected-command history.

These states describe a fitted command filter, not measured aircraft rates.
Only roll/pitch have an identified second-order-plus-zero response. The yaw
first-order time constant is explicitly marked as an engineering assumption.
There is no measurement filter, inferred transport delay, or gyro observer.
"""
from __future__ import annotations

from bisect import bisect_right
from collections import OrderedDict
from collections.abc import Mapping
from dataclasses import dataclass
import math

import numpy as np
from scipy.linalg import expm


def _number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be finite and in [{low}, {high}]')
    return float(value)


def validate_configuration(configuration=None):
    """Return a normalized explicit model; an omitted model remains ideal/off."""
    if configuration is None:
        configuration = {}
    if not isinstance(configuration, Mapping) or set(configuration) - {'enabled', 'roll', 'pitch', 'yaw'}:
        raise ValueError('Unknown INDI rate-response configuration')
    enabled = configuration.get('enabled', False)
    if type(enabled) is not bool:
        raise ValueError('rate-response enabled must be boolean')
    if not enabled:
        if set(configuration) - {'enabled'}:
            raise ValueError('Disabled rate response cannot contain unused axis settings')
        return {'enabled': False}
    result = {'enabled': True}
    for axis in ('roll', 'pitch'):
        p = configuration.get(axis)
        if not isinstance(p, Mapping) or set(p) != {'wn_rad_s', 'zeta', 'zero_s'}:
            raise ValueError(f'Explicit {axis} second-order model required')
        result[axis] = {
            'wn_rad_s': _number(p['wn_rad_s'], axis + ' wn_rad_s', 5., 60.),
            'zeta': _number(p['zeta'], axis + ' zeta', .15, 3.),
            'zero_s': _number(p['zero_s'], axis + ' zero_s', 0., .2),
        }
    p = configuration.get('yaw')
    if not isinstance(p, Mapping) or set(p) != {'tau_s', 'identified'} or type(p['identified']) is not bool:
        raise ValueError('Explicit yaw tau_s and identified boolean required')
    result['yaw'] = {'tau_s': _number(p['tau_s'], 'yaw tau_s', .01, .5), 'identified': p['identified']}
    return result


def continuous_matrices(configuration):
    """Return A(6,6), B(6,3), C(3,6), states ordered x1,x2 per body axis.

    For each roll/pitch axis x1dot=x2,
    x2dot=wn²(u-x1)-2*zeta*wn*x2, omega=x1+zero_s*x2.
    This proper realization does not differentiate a stepped rate command.
    Yaw's unused second state decays, and its measured output is x1 only.
    """
    c = validate_configuration(configuration)
    if not c['enabled']:
        raise ValueError('Continuous dynamics require an enabled explicit rate model')
    a = np.zeros((6, 6)); b = np.zeros((6, 3)); output = np.zeros((3, 6))
    for index, axis in enumerate(('roll', 'pitch')):
        p = c[axis]; i = 2 * index; wn = p['wn_rad_s']
        a[i:i+2, i:i+2] = [[0., 1.], [-wn*wn, -2*p['zeta']*wn]]
        b[i+1, index] = wn*wn
        output[index, i:i+2] = [1., p['zero_s']]
    tau = c['yaw']['tau_s']
    a[4, 4] = a[5, 5] = -1. / tau; b[4, 2] = 1. / tau; output[2, 4] = 1.
    return a, b, output


def discrete_matrices(configuration, dt_s):
    """Exact ZOH discretization; dt=0 gives identity and no input effect."""
    dt = _number(dt_s, 'rate model dt_s', 0., 2.)
    a, b, output = continuous_matrices(configuration)
    augmented = np.zeros((9, 9)); augmented[:6, :6] = a; augmented[:6, 6:] = b
    transition = expm(augmented * dt)
    return transition[:6, :6], transition[:6, 6:], output


def response(configuration, frequencies_hz):
    """Analytic complex gain, shape (...,3); excludes host/ESP transport."""
    c = validate_configuration(configuration)
    f = np.asarray(frequencies_hz, dtype=float)
    if not np.isfinite(f).all() or np.any(f < 0):
        raise ValueError('Finite nonnegative frequencies required')
    if not c['enabled']:
        return np.ones(f.shape + (3,), dtype=complex)
    s = 2j * np.pi * f; answer = np.empty(f.shape + (3,), dtype=complex)
    for index, axis in enumerate(('roll', 'pitch')):
        p = c[axis]; wn = p['wn_rad_s']
        answer[..., index] = wn*wn*(1.+p['zero_s']*s)/(s*s+2*p['zeta']*wn*s+wn*wn)
    answer[..., 2] = 1. / (1.+c['yaw']['tau_s']*s)
    return answer


def _stamp(value):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 0:
        raise ValueError('Nonnegative integer timestamp in host monotonic ns required')
    return int(value)


def _vector(value, size, name, bound):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all() or np.any(abs(result) > bound):
        raise ValueError(f'Invalid bounded {name}')
    return result.copy()


@dataclass
class _Checkpoint:
    stamp_ns: int
    state: np.ndarray
    rates: np.ndarray


class RateModelHistory:
    """Causal nominal filter state from actual selected body-FLU commands.

    reset(stamp_ns) may explicitly seed a known-static aircraft. Every later
    commit(stamp_ns, selected_rates) keeps a state checkpoint and the input
    held from that timestamp. state_at/output_at never modify history.

    Reading before retained history or too far beyond the last commit raises;
    an unavailable state is never silently replaced by a zero state. Predicted
    history contains no FC gyro assimilation. Measured position/attitude/
    velocity remain feedback inputs to MPC independently of this filter.
    """
    def __init__(self, configuration, *, history_s=2., max_prediction_s=.25):
        self.configuration = validate_configuration(configuration)
        if not self.configuration['enabled']:
            raise ValueError('Command history requires an enabled explicit rate model')
        self.history_s = _number(history_s, 'history_s', .25, 10.)
        self.max_prediction_s = _number(max_prediction_s, 'max_prediction_s', 0., .5)
        self.a, self.b, self.c = continuous_matrices(self.configuration)
        self._events = []; self._matrices = OrderedDict()

    @property
    def initialized(self):
        return bool(self._events)

    @property
    def earliest_stamp_ns(self):
        return self._events[0].stamp_ns if self._events else None

    @property
    def latest_stamp_ns(self):
        return self._events[-1].stamp_ns if self._events else None

    def _checked_state(self, state):
        x = _vector(state, 6, 'rate model state', 500.)
        if np.any(abs(x[::2]) > 10.) or np.any(abs(self.c @ x) > 12.):
            raise ValueError('Rate model state exceeds prediction bounds')
        return x

    def reset(self, stamp_ns, state=None, rates=None):
        stamp = _stamp(stamp_ns)
        x = self._checked_state(np.zeros(6) if state is None else state)
        u = _vector(np.zeros(3) if rates is None else rates, 3, 'selected body rates', 10.)
        self._events = [_Checkpoint(stamp, x, u)]

    def _at(self, stamp_ns):
        stamp = _stamp(stamp_ns)
        if not self._events or stamp < self._events[0].stamp_ns:
            raise ValueError('Rate model timestamp precedes available command history')
        if stamp - self._events[-1].stamp_ns > round(self.max_prediction_s*1e9):
            raise ValueError('Rate model prediction exceeds maximum held-command horizon')
        index = bisect_right([event.stamp_ns for event in self._events], stamp)-1
        return stamp, self._events[index]

    def _transition(self, elapsed_ns):
        if elapsed_ns not in self._matrices:
            augmented = np.zeros((9, 9)); augmented[:6, :6] = self.a; augmented[:6, 6:] = self.b
            transition = expm(augmented * (elapsed_ns*1e-9))
            self._matrices[elapsed_ns] = (transition[:6, :6], transition[:6, 6:])
            if len(self._matrices) > 128:
                self._matrices.popitem(last=False)
        else:
            self._matrices.move_to_end(elapsed_ns)
        return self._matrices[elapsed_ns]

    def state_at(self, stamp_ns):
        stamp, event = self._at(stamp_ns)
        a, b = self._transition(stamp-event.stamp_ns)
        return self._checked_state(a @ event.state + b @ event.rates)

    def output_at(self, stamp_ns):
        return self.c @ self.state_at(stamp_ns)

    def command_at(self, stamp_ns):
        _, event = self._at(stamp_ns)
        return event.rates.copy()

    def commit(self, stamp_ns, selected_rates):
        stamp = _stamp(stamp_ns)
        rates = _vector(selected_rates, 3, 'selected body rates', 10.)
        if not self._events:
            raise ValueError('Explicit static reset required before selected-command commit')
        if stamp < self._events[-1].stamp_ns:
            raise ValueError('Selected command timestamp moved backwards')
        x = self.state_at(stamp)
        if stamp == self._events[-1].stamp_ns:
            self._events[-1] = _Checkpoint(stamp, x, rates)
        else:
            self._events.append(_Checkpoint(stamp, x, rates))
        cutoff = stamp-round(self.history_s*1e9)
        while len(self._events) > 1 and self._events[1].stamp_ns < cutoff:
            self._events.pop(0)

    def diagnostics(self):
        return {'initialized': self.initialized, 'earliest_stamp_ns': self.earliest_stamp_ns,
                'latest_stamp_ns': self.latest_stamp_ns, 'checkpoints': len(self._events),
                'source': 'nominal selected-command response; no rate measurement assimilation',
                'yaw_identified': self.configuration['yaw']['identified']}
