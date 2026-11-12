"""Causal, experimental cable evidence and shadow decisions; no control API."""
import math
import statistics

import numpy as np
from scipy.spatial.transform import Rotation


DEFAULTS = dict(mode='disabled', cable_length_source='physical_config', source_ip='10.0.0.109', device_id=9, bind='0.0.0.0', port=7460,
    max_source_gap_s=.1, max_receive_age_s=.25, max_queue_age_s=.1,
    baseline_capture_s=2., baseline_max_age_s=30., baseline_noise_floor_counts=100.,
    baseline_drift_bound_counts_per_s=None, maximum_baseline_uncertainty_counts=None,
    loading_polarity=None, taut_on_counts=None, slack_off_counts=None,
    filter_tau_s=.05, taut_dwell_s=.2, slack_dwell_s=.2, settled_taut_s=.5,
    geometry_uncertainty_m=.012, cable_length_uncertainty_m=.005,
    maximum_pair_skew_s=.012, geometry_max_age_s=.04,
    relative_speed_bound_m_s=None, fusion_timing_uncertainty_s=None,
    slack_geometry_margin_m=.01, payload_clearance_m=.03,
    ground_height_uncertainty_m=.015, path_clear_confirmed=False)


def configuration(settings=None):
    settings=dict(settings or {})
    if set(settings)-set(DEFAULTS):raise ValueError('Unknown cable engagement settings: '+str(sorted(set(settings)-set(DEFAULTS))))
    c={**DEFAULTS,**settings}
    if c['mode'] not in ('disabled','shadow'):raise ValueError('Cable engagement supports disabled or shadow only; active supervision is not implemented')
    if c['cable_length_source'] not in ('physical_config','hover_calibration'):raise ValueError('Choose physical_config or hover_calibration cable length')
    if type(c['device_id']) is not int or not 0<=c['device_id']<=255 or type(c['port']) is not int or not 0<c['port']<65536:raise ValueError('Invalid strain device/port')
    for key in ('source_ip','bind'):
        if not isinstance(c[key],str) or not c[key]:raise ValueError('Explicit strain source/bind required')
    if type(c['path_clear_confirmed']) is not bool:raise ValueError('path_clear_confirmed must be boolean')
    if c['loading_polarity'] not in (None,-1,1) or isinstance(c['loading_polarity'],bool):raise ValueError('Verified loading polarity must be -1 or +1')
    excluded={'mode','cable_length_source','source_ip','device_id','bind','port','loading_polarity','path_clear_confirmed'}
    optional={'baseline_drift_bound_counts_per_s','maximum_baseline_uncertainty_counts','taut_on_counts','slack_off_counts','relative_speed_bound_m_s','fusion_timing_uncertainty_s'}
    for key in set(c)-excluded:
        if c[key] is None and key in optional:continue
        if type(c[key]) not in (int,float) or not math.isfinite(c[key]) or c[key]<0:raise ValueError('Finite nonnegative '+key+' required')
        if key not in optional and c[key]==0:raise ValueError('Positive '+key+' required')
    if c['baseline_capture_s']>30:raise ValueError('Baseline capture must be at most 30 seconds')
    if c['taut_on_counts'] is not None and c['slack_off_counts'] is not None and not c['taut_on_counts']>c['slack_off_counts']>=0:raise ValueError('Taut threshold must exceed slack threshold')
    return c


def raw_geometry(physical, poses, now_ros_ns, settings, floor_height_m):
    """Unfiltered accepted body poses, never the projected pendulum direction.

    Each pose: stamp_ns, position (calibrated body origin), quaternion_xyzw,
    orientation_held. Bounds supplied here are diagnostic configuration, not
    an assertion of measured sensor/clock accuracy.
    """
    c=settings
    result={'valid':False,'clock_alignment_verified':False,'reasons':[]}
    try:
        a,p=poses['aircraft'],poses['payload']
        ages=[(now_ros_ns-x['stamp_ns'])/1e9 for x in (a,p)]
        skew=abs(a['stamp_ns']-p['stamp_ns'])/1e9
        result.update(aircraft_age_s=ages[0],payload_age_s=ages[1],pair_skew_s=skew)
        if any(not 0<=v<c['geometry_max_age_s'] for v in ages):raise ValueError('raw_mocap_stale_or_future')
        if skew>c['maximum_pair_skew_s']:raise ValueError('raw_mocap_pair_skew')
        if a['orientation_held'] or p['orientation_held']:raise ValueError('raw_mocap_orientation_held')
        positions=[np.asarray(x['position'],dtype=float) for x in (a,p)]
        quats=[np.asarray(x['quaternion_xyzw'],dtype=float) for x in (a,p)]
        if any(x.shape!=(3,) or not np.isfinite(x).all() for x in positions) or any(x.shape!=(4,) or not np.isfinite(x).all() or abs(np.linalg.norm(x)-1)>.001 for x in quats):raise ValueError('invalid_raw_pose')
        ra,rp=[Rotation.from_quat(x) for x in quats]
        attachment=positions[0]+ra.apply(physical['attachment_body_m'])
        hook=positions[1]+rp.apply([0.,0.,physical['payload_com_offset_below_hook_m']])
        distance=float(np.linalg.norm(hook-attachment));length=float(physical['cable_length_m'])
        half=np.asarray(physical['payload_size_m'])/2
        bottom=float(positions[1][2]-np.abs(rp.as_matrix()[2])@half)
        result.update(attachment_world_m=attachment.tolist(),hook_world_m=hook.tolist(),hook_distance_m=distance,cable_length_m=length,length_source='Physical configuration frozen at supervisor startup; no projected/hover-fit length',residual_m=distance-length,cube_bottom_m=bottom,floor_height_m=float(floor_height_m),payload_clearance_m=bottom-floor_height_m)
        timing=c['fusion_timing_uncertainty_s'];speed=c['relative_speed_bound_m_s']
        if timing is None or speed is None:raise ValueError('fusion_timing_or_motion_bound_unconfigured')
        result.update(uncertainty_m=c['geometry_uncertainty_m']+c['cable_length_uncertainty_m']+speed*(skew+max(ages)+timing),valid=True)
    except (KeyError,ValueError,TypeError) as error:result['reasons'].append(str(error))
    return result


class EngagementEstimator:
    """Uses a supplied HX711Health instance. All times are host monotonic ns."""
    def __init__(self, health, settings=None):
        self.cfg=configuration(settings);self.health=health
        self.baseline=None;self.capture=None;self.filtered=None;self.sample_ns=None
        self.state='UNKNOWN';self.candidate=None;self.candidate_since=None;self.taut_since=None
        self.evidence_ns=None;self.last_evaluated_ns=None

    def clear_transition(self):
        self.state='UNKNOWN';self.candidate=None;self.candidate_since=None;self.taut_since=None;self.last_evaluated_ns=None

    def confirm_unloaded(self, now_ns):
        """Caller explicitly confirms ZERO external sensor load for capture."""
        h=self.health.check(now_ns)
        if not h['health_ok']:return False,'Healthy live strain stream required before unloaded confirmation'
        self.baseline=None;self.filtered=None;self.clear_transition()
        self.capture={'generation':h['baseline_generation'],'confirmation_ns':now_ns,'samples':[]}
        return True,'Unloaded confirmation accepted; maintain zero external load throughout baseline capture'

    def observe(self, packet, now_ns):
        result=self.health.observe(packet,now_ns)
        if result['baseline_invalidated']:
            self.baseline=None;self.capture=None
        if not result['sample_usable']:
            self.filtered=None;self.clear_transition()
            # A discontinuity cancels capture; it does not silently extend consent.
            self.capture=None
            return result
        if self.capture is not None:
            capture=self.capture
            if capture['generation']!=result['baseline_generation']:
                self.capture=None
            else:
                capture['samples'].append((now_ns,float(packet['raw'])))
                if (now_ns-capture['confirmation_ns'])/1e9>=self.cfg['baseline_capture_s']:
                    samples=capture['samples']
                    if len(samples)>=20:
                        times=np.array([(x[0]-now_ns)/1e9 for x in samples]);values=np.array([x[1] for x in samples])
                        self.baseline=dict(counts=float(statistics.median(values)),noise_counts=max(self.cfg['baseline_noise_floor_counts'],float(np.std(values))),capture_slope_counts_per_s=float(np.polyfit(times,values,1)[0]),stamp_ns=now_ns,confirmation_ns=capture['confirmation_ns'],generation=capture['generation'],samples=len(samples),evidence='Explicit operator confirmation of zero external load; never inferred from floor/ADC')
                    self.capture=None
        dt=None if self.sample_ns is None else (now_ns-self.sample_ns)/1e9
        alpha=1. if self.filtered is None or dt is None else -math.expm1(-dt/self.cfg['filter_tau_s'])
        self.filtered=float(packet['raw']) if self.filtered is None else self.filtered+alpha*(packet['raw']-self.filtered)
        self.sample_ns=now_ns
        return result

    def invalidate(self, reason):
        self.health.invalidate(reason);self.filtered=None;self.capture=None;self.clear_transition()

    def evaluate(self, now_ns, geometry, phase='waiting', path_clear=False):
        h=self.health.check(now_ns);reasons=[];c=self.cfg
        b=self.baseline
        diagnostic=dict(mode='shadow',state='UNKNOWN',mission_phase=phase,health=h,baseline=b,baseline_age_s=None,baseline_uncertainty_counts=None,filtered_counts=self.filtered,strain_delta_counts=None,strain_lower_counts=None,strain_upper_counts=None,geometry=geometry,proposed_transition=None,control_affected=False,thresholds_validated=False,baseline_capture_active=self.capture is not None)
        if not h['health_ok']:reasons.extend(h['last_reasons'] or ['strain_stale'])
        if b is None:reasons.append('verified_unloaded_baseline_missing')
        elif b['generation']!=h['baseline_generation']:
            self.baseline=None;reasons.append('baseline_generation_invalidated')
        else:
            age=(now_ns-b['stamp_ns'])/1e9;diagnostic['baseline_age_s']=age
            if not 0<=age<=c['baseline_max_age_s']:reasons.append('baseline_expired')
            bound=c['baseline_drift_bound_counts_per_s']
            if bound is None or c['maximum_baseline_uncertainty_counts'] is None:reasons.append('baseline_uncertainty_bound_unconfigured')
            else:
                uncertainty=3*b['noise_counts']+max(bound,abs(b['capture_slope_counts_per_s']))*max(age,0)
                diagnostic['baseline_uncertainty_counts']=uncertainty
                if uncertainty>c['maximum_baseline_uncertainty_counts']:reasons.append('baseline_uncertainty_exceeded')
                if self.filtered is not None and c['loading_polarity'] is not None:
                    delta=c['loading_polarity']*(self.filtered-b['counts'])
                    diagnostic.update(strain_delta_counts=delta,strain_lower_counts=delta-uncertainty,strain_upper_counts=delta+uncertainty)
        if any(c[key] is None for key in ('loading_polarity','taut_on_counts','slack_off_counts')):reasons.append('polarity_or_thresholds_unconfigured')
        if not geometry.get('valid',False):reasons.extend(geometry.get('reasons') or ['raw_geometry_missing'])
        if self.filtered is None:reasons.append('filtered_strain_missing')
        wanted=None
        if not reasons:
            residual=geometry['residual_m'];unc=geometry['uncertainty_m']
            near=abs(residual)<=unc
            short=residual<-(unc+c['slack_geometry_margin_m'])
            lower,upper=diagnostic['strain_lower_counts'],diagnostic['strain_upper_counts']
            high=lower>c['taut_on_counts'] or (self.state=='TAUT' and lower>c['slack_off_counts'])
            low=abs(diagnostic['strain_delta_counts'])+diagnostic['baseline_uncertainty_counts']<c['slack_off_counts']
            if high and near:wanted='TAUT'
            elif low and short:wanted='SLACK'
            elif high and short:reasons.append('tension_with_short_path_possible_snag')
            elif residual>unc:reasons.append('cable_overlength_or_geometry_inconsistent')
            elif low and near:reasons.append('low_strain_at_full_length_cannot_prove_slack')
            else:reasons.append('strain_or_geometry_ambiguous')
        if wanted is None:self.clear_transition()
        else:
            # Persistence advances only when a new healthy packet was observed.
            if self.sample_ns!=self.last_evaluated_ns:
                if wanted!=self.candidate:
                    self.candidate=wanted;self.candidate_since=self.sample_ns
                    if wanted!='TAUT':self.taut_since=None
                dwell=c['taut_dwell_s'] if wanted=='TAUT' else c['slack_dwell_s']
                if (self.sample_ns-self.candidate_since)/1e9>=dwell:
                    self.state=wanted
                    if wanted=='TAUT' and self.taut_since is None:self.taut_since=self.sample_ns
                else:self.state='UNKNOWN';reasons.append('transition_dwell_pending')
                self.last_evaluated_ns=self.sample_ns
            elif self.state=='UNKNOWN':reasons.append('transition_dwell_pending')
        diagnostic.update(state=self.state,reasons=reasons,taut_duration_s=0. if self.taut_since is None else max(0.,(self.sample_ns-self.taut_since)/1e9))
        clearance=geometry.get('payload_clearance_m')
        grounded=clearance is not None and abs(clearance)<=c['ground_height_uncertainty_m']
        airborne=clearance is not None and clearance>c['ground_height_uncertainty_m']
        diagnostic['payload_support_state']='grounded_geometry_only' if grounded else 'airborne_geometry_only' if airborne else 'unknown'
        if phase in ('place_payload','reposition','landing','landed'):
            proposed='preserve_existing_landing_and_payload_compensation'
        elif self.state=='UNKNOWN':
            proposed='pause_take_up_grounded' if grounded else 'controlled_airborne_recovery_keep_payload_compensation' if airborne else 'pause_take_up_support_unknown'
        elif not path_clear:proposed='pause_take_up_path_unverified'
        elif self.state=='SLACK':proposed='permit_slow_take_up'
        elif diagnostic['taut_duration_s']<c['settled_taut_s']:proposed='pause_take_up_settle'
        elif clearance is not None and clearance>=c['payload_clearance_m']:proposed='permit_mpc_handover_subject_to_existing_checks'
        else:proposed='permit_slow_lift_subject_to_existing_checks'
        diagnostic['proposed_transition']=proposed
        return diagnostic
