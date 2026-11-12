from collections import deque
import math


class HoverQualification:
    def __init__(self, options=None):
        options=options or {}
        allowed={'rate_filter_tau_s','minimum_rate_good_fraction','maximum_continuous_rate_exceedance_s','rate_limit_rad_s','maximum_command_rate_rad_s'}
        if set(options)-allowed:raise ValueError('Unknown hover qualification option')
        self.tau=float(options.get('rate_filter_tau_s',.05))
        self.minimum_fraction=float(options.get('minimum_rate_good_fraction',.95))
        self.maximum_exceedance=float(options.get('maximum_continuous_rate_exceedance_s',.1))
        self.limit=float(options.get('rate_limit_rad_s',.5));self.command_limit=float(options.get('maximum_command_rate_rad_s',.5))
        if not all(math.isfinite(v) for v in (self.tau,self.minimum_fraction,self.maximum_exceedance,self.limit,self.command_limit)) or not .02<=self.tau<=.1 or not .75<=self.minimum_fraction<=1. or not .04<=self.maximum_exceedance<=.1 or not .5<=self.limit<=1. or not .5<=self.command_limit<=1.:
            raise ValueError('Hover qualification requires filter20..100ms, good fraction75..100%, sustained limit40..100ms, measured/command rate limits0.5..1rad/s')
        self.window_ns=2_000_000_000
        self.reset()


    def reset(self):
        self.intervals=deque()
        self.last_now_ns=self.last_source_ns=None
        self.filtered_rate=None;self.previous_good=False
        self.raw_high_since_ns=self.other_ready_since_ns=None


    def update(self, now_ns, source_ns, raw_rate, checks):
        now_ns,source_ns=int(now_ns),int(source_ns)
        blockers=[name for name,passed in checks.items() if not passed]
        data_error=None
        if not math.isfinite(raw_rate) or raw_rate<0:data_error='invalid_aircraft_rate'
        elif now_ns<=0 or source_ns<=0 or not -2_000_000<=now_ns-source_ns<40_000_000:data_error='aircraft_measurement_age'
        elif self.last_now_ns is not None and now_ns<=self.last_now_ns:data_error='qualification_clock_reversal'
        elif self.last_source_ns is not None and source_ns<=self.last_source_ns:data_error='repeated_or_reversed_source'
        if data_error:
            self.reset()
            return {'ready':False,'blockers':blockers+[data_error],'checks':checks,'raw_rate_norm_rad_s':float(raw_rate) if math.isfinite(raw_rate) else None,'filtered_rate_norm_rad_s':None,'rate_good_fraction':0.,'window_covered_s':0.,'other_ready_s':0.,'raw_exceedance_s':0.,'history_reset':data_error}
        history_reset=None
        if self.last_now_ns is not None and (now_ns-self.last_now_ns>40_000_000 or source_ns-self.last_source_ns>40_000_000):
            self.reset();history_reset='observation_or_control_gap'
        if self.filtered_rate is None:self.filtered_rate=float(raw_rate)
        else:
            alpha=-math.expm1(-(source_ns-self.last_source_ns)/1e9/self.tau)
            self.filtered_rate+=alpha*(raw_rate-self.filtered_rate)
        filtered_good=self.filtered_rate<self.limit
        if raw_rate>=self.limit:
            if self.raw_high_since_ns is None:self.raw_high_since_ns=now_ns
        else:self.raw_high_since_ns=None
        exceedance=0. if self.raw_high_since_ns is None else (now_ns-self.raw_high_since_ns)/1e9
        sustained=exceedance>=self.maximum_exceedance
        if sustained:
            self.intervals.clear();history_reset='sustained_raw_rate'
        elif self.last_now_ns is not None:
            self.intervals.append((self.last_now_ns,now_ns,self.previous_good and filtered_good))
        cutoff=now_ns-self.window_ns
        while self.intervals and self.intervals[0][1]<=cutoff:self.intervals.popleft()
        if self.intervals and self.intervals[0][0]<cutoff:
            begin,end,good=self.intervals[0];self.intervals[0]=(cutoff,end,good)
        covered=sum(end-begin for begin,end,good in self.intervals)
        good_ns=sum(end-begin for begin,end,good in self.intervals if good)
        fraction=good_ns/covered if covered else 0.
        other_good=all(checks.values())
        self.other_ready_since_ns=(now_ns if self.other_ready_since_ns is None else self.other_ready_since_ns) if other_good else None
        other_s=0. if self.other_ready_since_ns is None else (now_ns-self.other_ready_since_ns)/1e9
        if covered<self.window_ns:blockers.append('rate_window_warming')
        if fraction+1e-12<self.minimum_fraction:blockers.append('rate_window_good_fraction')
        if sustained:blockers.append('sustained_aircraft_rate')
        if raw_rate>=self.limit:blockers.append('current_aircraft_rate')
        if not filtered_good:blockers.append('filtered_aircraft_rate')
        if other_s<2.:blockers.append('other_conditions_dwell')
        self.last_now_ns,self.last_source_ns=now_ns,source_ns
        self.previous_good=filtered_good and not sustained
        return {'ready':not blockers,'blockers':blockers,'checks':checks,'raw_rate_norm_rad_s':float(raw_rate),'filtered_rate_norm_rad_s':float(self.filtered_rate),'rate_limit_rad_s':self.limit,'maximum_command_rate_rad_s':self.command_limit,'rate_filter_tau_s':self.tau,'rate_good_fraction':float(fraction),'minimum_rate_good_fraction':self.minimum_fraction,'window_covered_s':covered/1e9,'window_required_s':2.,'other_ready_s':float(other_s),'raw_exceedance_s':float(exceedance),'maximum_continuous_rate_exceedance_s':self.maximum_exceedance,'history_reset':history_reset}
