"""Log TF acceptance and state expiry without granting control authority.

Raw inputs are logged at their actual callback rate, independently of the
25 Hz selected-state forwarder. Counters and saved poses are diagnostic only;
Rejected poses never enter control. A still-fresh, previously accepted packet
may be retained without changing its source timestamp, receipt time or age.
"""
import math
from numbers import Real

from sanity_zenoh.mocap import MocapOutlier, state_age


def retain_fresh_packet_after_outlier(previous, error, ros_now_ns):
    """Reject a bad pose without discarding a valid one within the 40 ms limit.

    Return the original object: no retimestamping, extrapolation, retransmission
    or new sensor acceptance occurs. Other validation errors clear control as
    before; a genuinely expired or unready packet cannot receive grace.
    """
    if previous is None or not isinstance(error, MocapOutlier):
        return None
    try:
        state_age(previous, ros_now_ns)
    except ValueError:
        return None
    return previous


def finite_values(values):
    """An invalid sensor sample must still produce valid JSON evidence."""
    return [float(value) if math.isfinite(float(value)) else None for value in values]


def json_diagnostics(value):
    if isinstance(value, dict):
        return {key:json_diagnostics(item) for key,item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_diagnostics(item) for item in value]
    if isinstance(value, Real) and not isinstance(value, (int, bool)):
        return float(value) if math.isfinite(float(value)) else None
    return value


class MocapDiagnostics:
    def __init__(self):
        self.inputs = self.accepted = self.ignored = self.rejected = 0
        self.last_input = self.last_accepted = self.last_rejection = None

    def input(self, *, parent, child, stamp_ns, position, quaternion,
              ros_now_ns, monotonic_ns, packet=None, error=None):
        self.inputs += 1
        outcome = 'rejected' if error is not None else 'accepted' if packet is not None else 'ignored'
        event = dict(kind='mocap_tf_input', input_sequence=self.inputs,
                     parent_frame=parent, tracked_frame=child,
                     source_stamp_ns=int(stamp_ns), receipt_ros_ns=int(ros_now_ns),
                     arrival_monotonic_ns=int(monotonic_ns),
                     raw_source_age_s=(ros_now_ns-stamp_ns)*1e-9,
                     position_xyz_m=finite_values(position),
                     quaternion_xyzw=finite_values(quaternion), outcome=outcome)
        if outcome == 'accepted':
            self.accepted += 1
            event['estimator_ready'] = packet.get('ready') is True
            self.last_accepted = dict(source_stamp_ns=int(packet['stamp_ns']),
                                      receipt_ros_ns=int(packet['receipt_ns']),
                                      arrival_monotonic_ns=int(monotonic_ns))
        elif outcome == 'ignored':
            self.ignored += 1
            event['reason'] = 'nonincreasing_source_stamp'
        else:
            self.rejected += 1
            event.update(reason=str(error), error_type=type(error).__name__)
            details = getattr(error, 'diagnostics', None)
            if details is not None:
                # MocapOutlier's diagnostic numbers are finite when the raw pose
                # is valid; nonfinite raw poses are represented by null above.
                event['rejection_diagnostics'] = json_diagnostics(details)
            self.last_rejection = dict(source_stamp_ns=int(stamp_ns),
                                        arrival_monotonic_ns=int(monotonic_ns),
                                        reason=str(error), error_type=type(error).__name__)
        self.last_input = dict(source_stamp_ns=int(stamp_ns),
                               arrival_monotonic_ns=int(monotonic_ns), outcome=outcome)
        return event

    def snapshot(self, *, ros_now_ns, monotonic_ns, state_error=None, callback_error=None):
        def age_since(item):
            return None if item is None else (monotonic_ns-item['arrival_monotonic_ns'])*1e-9
        return dict(inputs=self.inputs, accepted=self.accepted, ignored=self.ignored,
                    rejected=self.rejected,
                    last_input_outcome=None if self.last_input is None else self.last_input['outcome'],
                    last_input_gap_s=age_since(self.last_input),
                    last_accepted_gap_s=age_since(self.last_accepted),
                    last_accepted_source_age_s=None if self.last_accepted is None else
                        (ros_now_ns-self.last_accepted['source_stamp_ns'])*1e-9,
                    last_accepted_source_stamp_ns=None if self.last_accepted is None else
                        self.last_accepted['source_stamp_ns'],
                    last_rejection=self.last_rejection,
                    control_state_error=state_error,
                    last_callback_error=callback_error)


def update_with_diagnostics(estimator, diagnostics, record, *, parent, child,
                            stamp_ns, position, quaternion, ros_now_ns, monotonic_ns):
    """Return the estimator's original result/error; record every selected input."""
    packet = error = None
    try:
        packet = estimator.update(parent, child, stamp_ns, position, quaternion,
                                  ros_now_ns, ros_now_ns)
    except ValueError as exc:
        error = exc
    record(diagnostics.input(parent=parent, child=child, stamp_ns=stamp_ns,
                             position=position, quaternion=quaternion,
                             ros_now_ns=ros_now_ns, monotonic_ns=monotonic_ns,
                             packet=packet, error=error))
    return packet, error
