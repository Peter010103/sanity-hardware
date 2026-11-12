import threading
import time
import numpy as np
from .client import SanitySender
from .transport import SanityTransport
from .firmware_collective import FirmwareCollective
from .protocol import CTBR,MAX_LEASE_S


class PlateCommandGroup:
    def __init__(self, config, enable_output, recorder):
        self.config=config;self.enable_output=enable_output;self.recorder=recorder
        self.mappings=[FirmwareCollective(d['calibration']) for d in config.drones]
        recorder(dict(kind='plate_transport_policy',command_lease_s=MAX_LEASE_S,receiver_status_max_age_s=MAX_LEASE_S,
            selection_watchdog_s=.06,fc_status_max_age_ms=150))
        self.transport=SanityTransport(**config.data['transport']);self.senders=[]
        self.stopped=threading.Event();self.last_selection=time.monotonic();self.error=None
        try:
            for drone,mapping in zip(config.drones,self.mappings):
                self.senders.append(SanitySender(device=drone['id'],ctbr_mapping=mapping,physical_units=True,
                    disabled_only=not enable_output,transport=self.transport,diagnostics=True,
                    on_event=lambda event,device=drone['id']:recorder(dict(event,device=device))))
        except BaseException:self.close();raise
        self.watchdog=threading.Thread(target=self.watch,name='plate_group_watchdog',daemon=True);self.watchdog.start()


    def ready(self, enabled=False):
        now=time.monotonic()
        if self.error:raise ValueError(self.error)
        for sender,mapping in zip(self.senders,self.mappings):
            status=sender.receiver_status()
            if status is None:raise ValueError(f'Receiver {sender.device} not ready: receiver status missing/expired')
            age=now-sender.status_time
            failures=[name for name,bad in [('status_stale',age>=MAX_LEASE_S),('session_mismatch',status.session!=sender.session_id),
                ('mode_mismatch',status.mode!=CTBR),('receiver_failsafe',status.failsafe),
                (f'enabled={status.enabled}_expected_{enabled}',enabled is not None and status.enabled!=enabled),
                ('fc_not_ready',not status.fc_ready),('fc_status_stale',status.fc_age_ms>=150),('calibration_mismatch',status.calibration_id!=mapping.calibration_id)] if bad]
            if failures:
                diagnostic=dict(device=sender.device,failed=failures,status_age_ms=round(age*1000,1),fc_age_ms=status.fc_age_ms,
                    receiver_session=status.session,expected_session=sender.session_id,enabled=status.enabled,failsafe=status.failsafe,
                    calibration_id=status.calibration_id,expected_calibration_id=mapping.calibration_id)
                self.recorder(dict(kind='plate_receiver_not_ready',**diagnostic))
                raise ValueError(f'Receiver {sender.device} not ready: '+', '.join(failures)+f'; status_age={age*1000:.1f}ms, fc_age={status.fc_age_ms}ms')
            voltage=status.voltage_mv/1000.
            if not enabled and not 7.6<=voltage<=8.5:raise ValueError(f'Receiver {sender.device} start voltage outside7.6..8.5V')
            mapping.set_voltage(voltage)
        return True


    def send(self, commands, enabled):
        commands=np.asarray(commands).reshape(len(self.senders),4)
        if not np.isfinite(commands).all():raise ValueError('Finite fleet CTBR required')
        if enabled and not self.enable_output:raise ValueError('Shadow command group cannot enable aircraft')
        if enabled:
            try:self.ready(enabled=True if np.any(commands[:,0]>0) else None)
            except Exception:
                self.stop('Fleet receiver interlock lost');raise
        selected=[]
        try:
            for sender,mapping,command in zip(self.senders,self.mappings,commands):
                command=command.copy();command[0]=min(command[0],mapping.limit(command[0]))
                sender.ctbr(enabled,float(command[0]),tuple(command[1:]),valid_for_s=MAX_LEASE_S)
                si=sender.si_values
                selected.append([si[3]/10000.,*(v/10000. for v in si[:3])] if enabled else [0.]*4)
            self.last_selection=time.monotonic()
        except BaseException:self.stop('Partial group selection failed');raise
        return np.array(selected).ravel()


    def stop(self, reason):
        self.error=reason
        for sender in self.senders:sender.invalidate(reason)
        self.recorder({'kind':'plate_group_stop','reason':reason})


    def watch(self):
        while not self.stopped.wait(.005):
            if any(s.enabled for s in self.senders) and time.monotonic()-self.last_selection>.06:
                self.stop('Fleet command selection older than60ms');return


    def close(self):
        self.stopped.set()
        if hasattr(self,'watchdog'):self.watchdog.join(timeout=1)
        for sender in self.senders:sender.close()
        self.transport.close()
