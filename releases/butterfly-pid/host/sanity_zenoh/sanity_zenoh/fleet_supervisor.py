class FleetSupervisor:
    def __init__(self, devices, duration_s, safety, on_event=lambda event: None):
        self.devices = tuple(devices)
        self.duration_s, self.safety, self.on_event = duration_s, safety, on_event
        self.takeoff_mode = safety.get('takeoff_mode','sequential')
        if self.takeoff_mode not in ('sequential','simultaneous'):
            raise ValueError('Unknown takeoff_mode')
        self.state = 'waiting'
        self.started = None
        self.epoch = None
        self.ready_since = None
        self.admitted = set()
        self.isolated = set()
        self.actions = []
        self.reason = None


    def emit(self, action, device=None, **details):
        self.actions.append({'action':action,'device':device,**details})
        self.on_event({'kind':'fleet_'+action,'device':device,**details})


    def group_land(self, reason):
        if self.state in ('landing','finished','emergency'):
            return
        self.state, self.reason = 'landing', reason
        self.emit('land', reason=reason)


    def abort(self, device, reason='operator_individual_abort'):
        if device not in self.devices:
            raise ValueError('Unknown fleet device')
        if self.state != 'running' or self.safety['individual_failure'] == 'group_land':
            self.group_land(reason)
        else:
            self.isolated.add(device)
            self.emit('land',device,reason=reason)


    def emergency(self, device=None):
        if device is not None and device not in self.devices:
            raise ValueError('Unknown fleet device')
        self.emit('stop',device,reason='operator_emergency_disarm')
        if device is None:
            self.state, self.reason = 'emergency','operator_emergency_disarm'
        else:
            self.abort(device,'individual_emergency_disarm')


    def drain(self):
        actions,self.actions = self.actions,[]
        return actions


    def update(self, now, reports):
        if self.started is None:self.started = now
        if self.state in ('finished','emergency'):
            return self.drain()
        for device in self.devices:
            report = reports[device]
            if report.get('fault') and device not in self.isolated:
                self.abort(device,report['fault'])
                self.isolated.add(device)
        if self.state == 'landing':
            if all(reports[d].get('done') for d in self.devices):self.state = 'finished'
            return self.drain()
        if now-self.started > self.safety['startup_timeout_s'] and self.state != 'running':
            self.group_land('fleet_startup_or_readiness_timeout')
            return self.drain()
        if self.state == 'waiting':
            if all(reports[d].get('preflight') for d in self.devices):self.state = 'takeoff'
            else:return self.drain()
        if self.state in ('takeoff','barrier'):
            # Concurrent admission releases all controllers on this supervision tick.
            # Each receiver still acknowledges arming independently; radio arrival is not synchronized.
            for device in self.devices:
                if device not in self.admitted:
                    if self.takeoff_mode == 'simultaneous' or all(reports[d].get('ready') for d in self.admitted):
                        self.admitted.add(device)
                        self.emit('start',device,takeoff_mode=self.takeoff_mode,release_monotonic_s=now)
                    if self.takeoff_mode == 'sequential':break
            all_ready = len(self.admitted)==len(self.devices) and all(reports[d].get('ready') for d in self.devices)
            if all_ready:
                self.state = 'barrier'
                if self.ready_since is None:self.ready_since = now
                if now-self.ready_since >= self.safety['ready_dwell_s']:
                    self.epoch = now+self.safety['start_lead_s']
                    self.state = 'scheduled'
                    self.emit('schedule',epoch_monotonic_s=self.epoch)
            else:self.ready_since = None
        elif self.state == 'scheduled':
            all_ready = all(reports[d].get('ready') for d in self.devices)
            all_ack = all(reports[d].get('epoch_ack') == self.epoch for d in self.devices)
            if not all_ready or (now>=self.epoch and not all_ack):
                self.emit('cancel_schedule',reason='participant_not_ready_or_acknowledged')
                self.epoch, self.ready_since, self.state = None,None,'barrier'
            elif now >= self.epoch:
                self.state = 'running'
                self.emit('mission_started',epoch_monotonic_s=self.epoch)
        elif self.state == 'running' and now>=self.epoch+self.duration_s:
            self.group_land('mission_complete')
        return self.drain()
