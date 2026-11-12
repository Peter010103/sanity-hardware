"""Classify a receiver heartbeat gap separately from explicit receiver faults."""
import math

STARTUP_STATUS_MAX_S = .150
AIRBORNE_STATUS_MAX_S = .300


def receiver_readiness(receiver, *, age_s, session, mode, controller,
                       airborne=False, controller_fresh=False, pose_fresh=False):
    result = dict(ready=False, reason='receiver_missing', landing_requested=False,
                  status_age_ms=age_s*1000 if math.isfinite(age_s) else None,
                  startup_max_age_ms=150, airborne_max_age_ms=300)
    if receiver is None:
        return result
    checks = [
        (math.isfinite(age_s) and age_s >= 0, 'receiver_clock_invalid'),
        (receiver.session == session and receiver.mode == mode, 'receiver_identity_changed'),
        (controller is not None and controller.get('receiver_epoch') == receiver.epoch, 'receiver_epoch_changed'),
        (not receiver.failsafe, 'receiver_failsafe'),
        (receiver.fc_ready and receiver.fc_age_ms < 150, 'receiver_fc_unavailable'),
        (6600 <= receiver.voltage_mv <= 8500, 'receiver_voltage_unavailable'),
    ]
    for good, reason in checks:
        if not good:
            result['reason'] = reason
            return result
    if age_s < STARTUP_STATUS_MAX_S:
        result.update(ready=True, reason='ok')
    elif (airborne and age_s < AIRBORNE_STATUS_MAX_S and
          controller_fresh and pose_fresh):
        result.update(ready=True, reason='receiver_status_delayed', landing_requested=True)
    else:
        result['reason'] = 'receiver_status_unavailable'
    return result
