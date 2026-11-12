"""Enter-confirmed receive-only test; observations never authorize flight."""
import math


class ShadowBenchProtocol:
    def __init__(self, cycles=3, hold_s=10., release_s=10., capture_s=2.):
        if type(cycles) is not int or not 1<=cycles<=20:raise ValueError('cycles must be 1..20')
        if any(not math.isfinite(x) or not .1<=x<=120 for x in (hold_s,release_s,capture_s)):raise ValueError('Test windows must be .1..120 seconds')
        self.cycles=cycles;self.hold_s=hold_s;self.release_s=release_s;self.capture_s=capture_s
        self.phase='waiting_data';self.trial=0;self.deadline=None;self.request_ns=None
        self.done=False;self.completed=False

    @property
    def needs_enter(self):return self.phase in ('setup_unloaded','setup_load','setup_release')

    def step(self, now_ns, diagnostic, pressed=False, confirm_unloaded=None):
        events=[]
        def move(phase,message,duration=None):
            self.phase=phase;self.deadline=None if duration is None else now_ns+round(duration*1e9)
            events.append(dict(kind='bench_test_phase',host_monotonic_ns=now_ns,phase=phase,trial=self.trial,message=message,operator_confirmed=pressed,physical_onset_measured=False))
        healthy=diagnostic.get('health',{}).get('health_ok',False)
        if self.phase=='waiting_data':
            if healthy:move('setup_unloaded','Support the drone AND payload so NO external load passes through the gauge. Press Enter when unloaded and hands off.')
        elif self.phase=='setup_unloaded' and pressed:
            ok,message=confirm_unloaded()
            if ok:
                self.request_ns=now_ns;move('capture_baseline','Keep the sensor unloaded: capturing baseline...',self.capture_s+3.)
            else:move('setup_unloaded',message+' — restore healthy data, keep unloaded, then press Enter to retry.')
        elif self.phase=='capture_baseline':
            baseline=diagnostic.get('baseline')
            if baseline is not None and baseline['confirmation_ns']>=self.request_ns and not diagnostic.get('baseline_capture_active',False):
                self.trial=1;move('setup_load','Keep the drone supported. Let the payload hang freely from the string; press Enter when steady and hands off.')
            elif now_ns>=self.deadline:
                move('setup_unloaded','Baseline capture did not complete. Fully unload the sensor and press Enter to retry.')
        elif self.phase=='setup_load' and pressed:
            move('loaded','RECORDING LOADED — leave the payload hanging, hands off.',self.hold_s)
        elif self.phase=='loaded' and now_ns>=self.deadline:
            move('setup_release','Support the payload and completely unload the gauge; press Enter when ready and hands off.')
        elif self.phase=='setup_release' and pressed:
            move('released','RECORDING UNLOADED — keep all sensor loads supported.',self.release_s)
        elif self.phase=='released' and now_ns>=self.deadline:
            if self.trial>=self.cycles:
                self.done=True;self.completed=True;move('complete','DONE — saving recordings and plots.')
            else:
                self.trial+=1;move('setup_load','Let the payload hang freely again, keeping the drone supported; press Enter when steady and hands off.')
        return events
