"""ROS-independent physical-command sender, with one latest command and no replay."""

from sanity_zenoh.transport import SanityTransport
from dataclasses import asdict
import json
import math
import secrets
import threading
import time

import zenoh
from udp_broadcaster.ctbr_protocol import CtbrMapping
from sanity_zenoh.fc_telemetry import decode_fc_telemetry
from sanity_zenoh.protocol import PAPER_NATIVE, PAPER_MODEL_ID, PaperDemand, paper_command_packet, CTBR, DIRECT, MAX_LEASE_MS, MAX_LEASE_S, ReceiverStatus, command_packet, neutral, ctbr_si_values, si_command_packet


class SanitySender:
    def __init__(
        self,
        device=9,
        mode=CTBR,
        listen="tcp/0.0.0.0:7447",
        connect=None,
        ctbr_mapping=None,
        direct_mapping=None,
        rate_hz=100.0,
        on_event=None,
        physical_units=False,
        telemetry_listen=None,
        telemetry_connect=None,
        diagnostics=False,
        disabled_only=False,
        transport=None,
        controller_selection=False,
    ):
        if (
            not 1 <= device <= 255
            or mode not in (CTBR, DIRECT, PAPER_NATIVE)
            or not math.isfinite(rate_hz)
            or not 1 <= rate_hz <= 100
        ):
            raise ValueError("Invalid device, mode or transmit rate (1..100 Hz)")
        self.on_event = on_event
        self.device, self.mode = device, mode
        self.ctbr_mapping = ctbr_mapping or CtbrMapping()
        self.direct_mapping = direct_mapping
        self.physical_units = bool(physical_units)
        if self.physical_units and (mode != CTBR or not getattr(self.ctbr_mapping,"calibration_id",0)):
            raise ValueError("Physical CTBR requires the shared ESP calibration profile")
        self.high_rate = bool(getattr(self.ctbr_mapping,"high_rate",False))
        self.controller_selection=bool(controller_selection)
        self.selected_high_rate=bool(getattr(self.ctbr_mapping,"selected_high_rate",False))
        if self.selected_high_rate and not self.controller_selection:raise ValueError("SZC6 high rates require explicit controller selection")
        if self.controller_selection and not self.physical_units:
            raise ValueError('Controller selection requires the standard physical CTBR interface')
        self.paper_demand=PaperDemand()
        self.paper_negotiated=False
        self.latest_paper_state=None
        self.paper_state_time=-math.inf
        self.indi_requested=False
        self.latest_controller_state=None
        self.controller_state_time=-math.inf
        if self.high_rate and not self.physical_units:
            raise ValueError("High-rate commands require the versioned physical interface")
        self.si_values = (0,0,0,0)
        self.rate_hz = rate_hz
        self.session_id = secrets.randbelow(2**32 - 1) + 1
        self.sequence = 0
        self.lock = threading.RLock()
        self.status = None
        self.status_time = -math.inf
        self.enabled = False
        self.values = neutral(mode)
        self.deadline = -math.inf
        self.rearm_required = True
        self.last_error = None
        self.stop_reason = None
        self.stopping = threading.Event()
        self.latest_telemetry = None
        self.telemetry_time = None
        self.telemetry_kT = getattr(self.ctbr_mapping,"profile",{}).get("effective_armed_kT_N_per_rad_s_squared")
        self.owns_transport = transport is None
        self.transport = transport or SanityTransport(listen,connect,telemetry_listen,telemetry_connect)
        try:
            self.transport.register(device)
        except BaseException:
            if self.owns_transport:
                self.transport.close()
            raise
        self.session = self.transport.session
        self.telemetry_session = self.transport.telemetry_session
        self.diagnostics = bool(diagnostics)
        self.disabled_only = bool(disabled_only)
        self.test_ack = None
        self.diagnostic_times = {}
        root = f"sanity/{device:02d}"
        try:
            self.subscriber = self.session.declare_subscriber(root + "/status", self.receive_status)
            telemetry_session = self.telemetry_session or self.session
            self.telemetry_subscriber = telemetry_session.declare_subscriber(root + "/telemetry/fc", self.receive_telemetry)
            if self.diagnostics:
                self.trace_subscriber=telemetry_session.declare_subscriber(root+"/diagnostics/trace",lambda sample:self.receive_diagnostic(sample,"trace"))
                self.pong_subscriber=self.session.declare_subscriber(root+"/diagnostics/pong",lambda sample:self.receive_diagnostic(sample,"pong"))
                self.test_subscriber=self.session.declare_subscriber(root+"/diagnostics/telemetry/ack",lambda sample:self.receive_diagnostic(sample,"test_config"))
                self.test_publisher=self.session.declare_publisher(root+"/diagnostics/telemetry/set",priority=zenoh.Priority.REAL_TIME,congestion_control=zenoh.CongestionControl.DROP,reliability=zenoh.Reliability.BEST_EFFORT,express=True)
                self.probe_publisher=self.session.declare_publisher(root+"/diagnostics/ping",priority=zenoh.Priority.REAL_TIME,congestion_control=zenoh.CongestionControl.DROP,reliability=zenoh.Reliability.BEST_EFFORT,express=True)
            self.publisher = self.session.declare_publisher(
                root + ("/command/paper_native" if mode == PAPER_NATIVE else "/command/ctbr" if mode == CTBR else "/command/direct_thrust"),
                priority=zenoh.Priority.REAL_TIME,
                congestion_control=zenoh.CongestionControl.DROP,
                reliability=zenoh.Reliability.BEST_EFFORT,
                express=True,
            )
        except BaseException:
            self.release_transport()
            raise
        self.thread = threading.Thread(
            target=self.run, name="sanity_zenoh_sender", daemon=True
        )
        self.thread.start()

    def receive_diagnostic(self, sample, kind):
        from sanity_zenoh.transport_test import decode_diagnostic
        raw=bytes(sample.payload);event={"kind":"zenoh_rx_"+kind,"payload_hex":raw.hex()}
        try:
            event.update(decode_diagnostic(raw,kind,self.device))
            if kind=='trace':
                from sanity_zenoh.command_timing import decode_command_trace
                event['command_timing']=[value for record in event['records']
                    if (value:=decode_command_trace(record)) is not None]
            self.diagnostic_times[kind] = time.monotonic()
            if kind=="test_config":
                with self.lock:self.test_ack=event.copy()
        except (ValueError,TypeError) as error:event["decode_error"]=str(error)
        if self.on_event:self.on_event(event)


    def configure_test_telemetry(self, enabled, nonce):
        from sanity_zenoh.transport_test import TEST_CONFIG
        if not self.diagnostics:raise ValueError("Diagnostics not enabled")
        with self.lock:
            if self.enabled:raise ValueError("Telemetry test configuration is disarmed-only")
            payload=TEST_CONFIG.pack(b"SZTM",1,self.device,int(bool(enabled)),0,self.session_id,nonce)
        self.test_publisher.put(payload)


    def invalidate(self, reason="Command expired"):
        with self.lock:
            was_enabled = self.enabled
            if was_enabled and self.stop_reason is None:
                self.stop_reason = reason
            if self.on_event and (was_enabled or reason != self.last_error):
                self.on_event({"kind":"sender_interlock","reason":reason,"primary_stop_reason":self.stop_reason,
                    "was_enabled":was_enabled,"status_age_ms":(time.monotonic()-self.status_time)*1000 if self.status is not None else None,
                    "command_deadline_remaining_ms":(self.deadline-time.monotonic())*1000 if self.enabled else None,
                    "receiver_flags_before_stop":self.status.flags if self.status else None,"receiver_sequence":self.status.sequence if self.status else None})
            self.enabled = False
            self.values = neutral(self.mode)
            self.si_values = (0,0,0,0)
            self.paper_demand=PaperDemand()
            self.paper_negotiated=False
            self.indi_requested = False
            self.deadline = -math.inf
            self.rearm_required = True
            self.last_error = reason

    def receive_status(self, sample):
        try:
            payload = bytes(sample.payload)
            if self.on_event:
                self.on_event({"kind":"zenoh_rx_status","payload_hex":payload.hex()})
            status = ReceiverStatus.decode(payload)
            if status.device != self.device:
                return
            with self.lock:
                if status.mode != self.mode and self.enabled:
                    self.invalidate("Receiver mode changed: send an explicit disabled command")
                if self.status is None or status.epoch != self.status.epoch:
                    self.invalidate(
                        "Receiver connected/restarted: send an explicit disabled command"
                    )
                elif (
                    status.receiver_ms - self.status.receiver_ms
                ) & 0xFFFFFFFF >= 0x80000000:
                    return
                if self.enabled and status.failsafe:
                    self.invalidate(
                        "Receiver watchdog/interlock stopped output; voltage model fallback/clamp active; explicit reset required" if not status.voltage_supported else "Receiver watchdog/interlock stopped output; explicit reset required"
                    )
                self.status, self.status_time = status, time.monotonic()
        except (ValueError, TypeError):
            return

    def receive_telemetry(self, sample):
        payload = bytes(sample.payload)
        event = {"kind":"zenoh_rx_fc_telemetry","payload_hex":payload.hex()}
        try:
            telemetry = decode_fc_telemetry(payload,self.device,self.telemetry_kT)
            self.latest_telemetry = telemetry
            self.telemetry_time = time.monotonic()
            for record in telemetry['records']:
                paper=record.get('paper_state')
                if paper is not None:
                    with self.lock:
                        query=record.get('query_write_begin_esp_us');parsed=record['response_parsed_esp_us']
                        if self.status is not None and record['epoch']==self.status.epoch and query is not None and 0<=parsed-query<100000:
                            self.latest_paper_state={**paper,'receiver_epoch':record['epoch'],'response_esp_ms':parsed//1000}
                            self.paper_state_time=time.monotonic()
                            if not paper['native_capable'] or paper['backend']!=1:
                                if self.mode==PAPER_NATIVE and self.enabled:self.invalidate('FC paper capability/configuration rejected')
                                self.paper_negotiated=False
                state=record.get('controller_state')
                if state is not None:
                    with self.lock:
                        if self.status is None or record['epoch'] != self.status.epoch:
                            continue
                        query=record.get('query_write_begin_esp_us')
                        parsed=record['response_parsed_esp_us']
                        if query is None or not 0 <= parsed-query < 100000:
                            continue
                        previous=self.latest_controller_state
                        if previous is not None and state['sequence']==previous['sequence']:
                            continue  # A repeated snapshot must not refresh controller freshness.
                        if previous is not None and ((state['fc_time_us']-previous['fc_time_us'])&0xffffffff)>0x80000000:
                            if self.enabled:self.invalidate('Flight controller clock reset/reordered state')
                            self.latest_controller_state=None
                            self.controller_state_time=-math.inf
                            continue
                        period=None
                        if previous is not None:
                            ticks=(state['sequence']-previous['sequence'])&0xffffffff
                            elapsed=(state['fc_time_us']-previous['fc_time_us'])&0xffffffff
                            if 0<ticks<0x80000000:period=elapsed/ticks
                        state={**state,'receiver_epoch':record['epoch'],'response_esp_ms':parsed//1000,
                            'control_period_us':period}
                        self.latest_controller_state=state
                        self.controller_state_time=time.monotonic()
            event["telemetry"] = telemetry
        except (ValueError, TypeError) as error:
            event["decode_error"] = str(error)
        if self.on_event:
            self.on_event(event)


    def receiver_status(self):
        with self.lock:
            if time.monotonic() - self.status_time >= MAX_LEASE_S:
                return None
            return self.status

    def set_output(self, enabled, values, valid_for_s):
        with self.lock:
            now = time.monotonic()
            if not math.isfinite(valid_for_s) or not 0 < valid_for_s <= MAX_LEASE_S:
                raise ValueError("Command validity must be in (0, 1.0] seconds")
            if self.enabled and now >= self.deadline:
                self.invalidate()
            if not enabled:
                self.stop_reason = None
                self.rearm_required = False
                values = neutral(self.mode)
            elif self.rearm_required:
                raise ValueError("Explicit disabled command required before enabling" + (": " + self.stop_reason if self.stop_reason else ""))
            elif not (
                (claim := self.receiver_status())
                and claim.mode == self.mode
                and claim.session == self.session_id
                and not claim.failsafe
            ):
                raise ValueError("Wait for a fresh disabled acknowledgement in the selected mode before enabling")
            elif values != neutral(self.mode):
                status = self.receiver_status()
                zero_thrust = self.mode in (CTBR,PAPER_NATIVE) and values[3] == 1000
                if not zero_thrust and (
                    not self.enabled
                    or status is None
                    or not status.enabled
                    or status.session != self.session_id
                ):
                    raise ValueError(
                        "Send enabled zero thrust and wait for receiver acknowledgement before positive thrust"
                    )
            self.enabled, self.values = bool(enabled), tuple(values)
            self.command_generation_ns = time.monotonic_ns()
            self.deadline = now + valid_for_s
            self.last_error = None

    def controller_status(self, maximum_age_s=.15):
        health=self.controller_diagnostics(maximum_age_s)
        return health['snapshot'] if health['fresh'] else None

    def controller_diagnostics(self, maximum_age_s=.15):
        """Explain stale/rejected status without granting it control authority."""
        with self.lock:
            now=time.monotonic();state=self.latest_controller_state
            result={'fresh':False,'reason':'no_snapshot','snapshot':dict(state) if state is not None else None,
                'host_receipt_age_ms':None,'esp_sample_age_ms':None,'maximum_age_ms':maximum_age_s*1000}
            if state is None:return result
            host_age=now-self.controller_state_time
            result['host_receipt_age_ms']=host_age*1000
            if self.status is None:result['reason']='no_receiver_status';return result
            if state['receiver_epoch']!=self.status.epoch:result['reason']='epoch_mismatch';return result
            # Compare ESP clocks as well as host receipt time: buffered network
            # data must not become fresh merely because it just arrived.
            elapsed_ms=(self.status.receiver_ms-state['response_esp_ms'])&0xffffffff
            if elapsed_ms>=0x80000000:elapsed_ms-=0x100000000
            age=elapsed_ms*.001+now-self.status_time
            result['esp_sample_age_ms']=age*1000
            if not 0<=host_age<maximum_age_s:result['reason']='stale_host_receipt';return result
            if age<-.05:result['reason']='receiver_clock_order';return result
            if age>=maximum_age_s:result['reason']='stale_esp_sample';return result
            result['fresh']=True;result['reason']='ok'
            return result

    def ctbr(self, armed, thrust_newtons, rates_rad_s, valid_for_s=MAX_LEASE_S, indi_requested=False):
        try:
            if self.mode != CTBR:
                raise ValueError("Sender is not in CTBR mode")
            with self.lock:
                if type(indi_requested) is not bool or (indi_requested and (not self.controller_selection or not armed)):
                    raise ValueError('INDI request requires enabled SZC4 controller selection')
                if self.physical_units and armed:
                    status=self.receiver_status()
                    if status is None or status.calibration_id!=self.ctbr_mapping.calibration_id:
                        raise ValueError("ESP physical-unit capability/calibration mismatch; update firmware or profile")
                    if self.high_rate and self.controller_selection and (not status.indi_pitch_loop_supported or status.session!=self.session_id):
                        raise ValueError("ESP lacks session-bound SZC6 INDI high-pitch acknowledgement")
                    if self.high_rate and not status.pitch_loop_supported:
                        raise ValueError("ESP lacks SZC3 pitch-loop capability; flash the new firmware first")
                    self.ctbr_mapping.set_voltage(status.voltage_mv/1000)
                    if thrust_newtons>0 and (not status.fc_ready or status.fc_age_ms>=150):
                        raise ValueError("Fresh FC status required by CTBR output")
                mapped = self.ctbr_mapping.convert(self.device, armed, thrust_newtons, tuple(rates_rad_s))
                self.set_output(armed,(mapped.roll,mapped.pitch,mapped.yaw,mapped.throttle),valid_for_s)
                self.si_values=ctbr_si_values(armed,thrust_newtons,tuple(rates_rad_s),self.high_rate,self.selected_high_rate) if self.physical_units else (0,0,0,0)
                self.indi_requested=indi_requested
        except (ValueError, TypeError) as error:
            self.invalidate(str(error))
            raise

    def paper_status(self,maximum_age_s=.15):
        with self.lock:
            state=self.latest_paper_state;now=time.monotonic()
            if state is None or self.status is None or now-self.paper_state_time>=maximum_age_s or state['receiver_epoch']!=self.status.epoch:return None
            age=(self.status.receiver_ms-state['response_esp_ms'])&0xffffffff
            if age>=0x80000000:age-=0x100000000
            if not -.05<=age*.001+now-self.status_time<maximum_age_s:return None
            return dict(state)

    def paper_native(self,enabled,thrust_n=0.,alpha_rad_s2=(0.,0.,0.),fallback_rate_rad_s=(0.,0.,0.),valid_for_s=MAX_LEASE_S,paper_requested=False):
        try:
            if self.mode!=PAPER_NATIVE:raise ValueError("Sender is not in paper native mode")
            demand=PaperDemand(thrust_n,tuple(alpha_rad_s2),tuple(fallback_rate_rad_s));demand.validate()
            with self.lock:
                if enabled and not (self.enabled and self.paper_negotiated):
                    cap=self.paper_status()
                    if cap is None or not cap['native_capable'] or cap['backend']!=1 or cap['model_contract_id']!=PAPER_MODEL_ID:raise ValueError("Fresh FC PIN1 paper backend capability required")
                    if self.status is None or cap['accepted_epoch']!=self.status.epoch or cap['accepted_session']!=self.session_id:raise ValueError("Wait for FC disabled native claim acknowledgement")
                # Validate disable and request rules even before any network TX.
                paper_command_packet(self.device,bool(enabled),1,1,0,0,1,demand,bool(paper_requested))
                self.set_output(enabled,(1500,1500,1500,1000 if thrust_n==0 else 1001),valid_for_s)
                self.paper_demand=demand;self.indi_requested=bool(paper_requested);self.paper_negotiated=bool(enabled)
        except (ValueError,TypeError) as error:
            self.invalidate(str(error));raise

    def direct_thrust(self, enabled, thrusts_newtons, valid_for_s=MAX_LEASE_S):
        try:
            if self.mode != DIRECT:
                raise ValueError("Sender is not in direct thrust mode")
            if len(thrusts_newtons) != 4 or not all(
                math.isfinite(t) and 0 <= t <= 0.8 + 1e-6 for t in thrusts_newtons
            ):
                raise ValueError("Four rotor thrusts in 0..0.8 N required")
            values = neutral(DIRECT)
            if enabled and any(thrusts_newtons):
                status = self.receiver_status()
                if status is None or not status.fc_ready or status.fc_age_ms >= 150:
                    raise ValueError(
                        "Fresh, disarmed and compatible FC status required for direct thrust"
                    )
                if self.direct_mapping is None:
                    raise ValueError(
                        "Actuator profile and explicit rotor-to-output mapping required"
                    )
                values = self.direct_mapping.convert(
                    thrusts_newtons, status.voltage_mv / 1000
                )
            self.set_output(enabled, values, valid_for_s)
        except (ValueError, TypeError) as error:
            self.invalidate(str(error))
            raise

    def transmit(self):
        build_started_ns=time.monotonic_ns()
        with self.lock:
            now = time.monotonic()
            status = self.status
            if status is None or now - self.status_time >= MAX_LEASE_S:
                self.invalidate("Receiver status/lease lost")
                return
            if now >= self.deadline:
                if self.enabled:
                    self.invalidate()
                enabled, values, ttl = False, neutral(self.mode), MAX_LEASE_MS
            else:
                enabled, values = self.enabled, self.values
                remaining_ms = int((self.deadline - now) * 1000)
                # The wire expiry is echoed_receiver_ms + ttl, rather than
                # receipt_time + ttl. Account for time already spent holding
                # that receiver timestamp so fresh commands do not expire early
                # merely because the status downlink is delayed. The anchor is
                # unchanged, and rounding down / the one-second cap can only
                # shorten the original host generation deadline. Unmeasured
                # downlink latency also makes receiver expiry earlier, not later.
                ttl = min(MAX_LEASE_MS, int((self.deadline - self.status_time) * 1000))
                if remaining_ms < 1 or ttl < 1:
                    self.invalidate()
                    enabled, values, ttl = False, neutral(self.mode), MAX_LEASE_MS
            if getattr(self,"disabled_only",False) and (enabled or tuple(values)!=neutral(self.mode) or (self.physical_units and any(self.si_values))):
                raise ValueError("Disabled-only transport test rejected output before publication")
            # Reflect a receiver timestamp; never advance or fabricate its clock.
            if self.mode!=PAPER_NATIVE:
                packet = command_packet(
                    self.device,
                    self.mode,
                    enabled,
                    status.epoch,
                    self.session_id,
                    self.sequence,
                    status.receiver_ms,
                    ttl,
                    values,
                )
            if self.physical_units:
                packet=si_command_packet(self.device,enabled,status.epoch,self.session_id,self.sequence,
                    status.receiver_ms,ttl,self.si_values if enabled else (0,0,0,0),self.ctbr_mapping.calibration_id,self.high_rate,
                    self.controller_selection,self.indi_requested if enabled else False)
            if self.mode==PAPER_NATIVE:
                if enabled and not self.paper_negotiated:
                    self.invalidate("Paper capability lost");enabled=False;values=neutral(self.mode)
                packet=paper_command_packet(self.device,enabled,status.epoch,self.session_id,self.sequence,status.receiver_ms,ttl,
                    self.paper_demand if enabled else PaperDemand(),self.indi_requested if enabled else False)
            published_sequence=self.sequence
            source_generation_ns=getattr(self,'command_generation_ns',None)
            lease_receiver_ms=status.receiver_ms
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        publication_started_ns=time.monotonic_ns()
        self.publisher.put(packet)
        publication_completed_ns=time.monotonic_ns()
        if self.on_event:
            self.on_event({"kind":"zenoh_tx_command","payload_hex":packet.hex(),"enabled":enabled,"values":values,"sequence":published_sequence,"ttl_ms":ttl,"wire_units":"PIN1_T_N_alpha_rad_s2_Frd_companion_rad_s" if self.mode==PAPER_NATIVE else ("rates_1e-3_thrust_1e-4" if self.high_rate else "SI_1e-4") if self.physical_units else "RC_or_motor_counts",
                "packet_build_started_ns":build_started_ns,"publication_started_ns":publication_started_ns,
                "publication_completed_ns":publication_completed_ns,"source_command_generation_ns":source_generation_ns,
                "echoed_receiver_lease_ms":lease_receiver_ms,"timing_clock":"host_monotonic_ns"})

    def run(self):
        while not self.stopping.is_set():
            try:
                self.transmit()
            except Exception as error:
                self.invalidate(f"Zenoh send failed: {error}")
            self.stopping.wait(1 / self.rate_hz)

    def close(self):
        if getattr(self, "closed", False):
            return
        self.closed = True
        self.stopping.set()
        self.thread.join()
        self.invalidate("Sender closed")
        self.shutdown_acknowledged = False
        self.shutdown_error = None
        self.shutdown_status = None
        first_stop_sequence = self.sequence
        deadline = time.monotonic() + MAX_LEASE_S + .25
        try:
            while True:
                try:
                    self.transmit()
                except Exception as error:
                    self.shutdown_error = f"STOP publication failed: {error}"
                with self.lock:
                    status = self.receiver_status()
                    if (status and status.mode == self.mode and status.session == self.session_id
                        and not status.enabled and not status.failsafe
                        and ((status.sequence - first_stop_sequence) & 0xFFFFFFFF) < 0x80000000):
                        self.shutdown_acknowledged = True
                        self.shutdown_error = None
                        self.shutdown_status = asdict(status)
                        break
                if time.monotonic() >= deadline:
                    self.shutdown_error = self.shutdown_error or "No disabled receiver acknowledgement before shutdown timeout"
                    break
                time.sleep(.01)
        finally:
            if self.on_event:
                self.on_event({"kind":"sender_shutdown", "disabled_acknowledged":self.shutdown_acknowledged,
                    "error":self.shutdown_error, "receiver_status":self.shutdown_status})
            self.release_transport()


    def release_transport(self):
        try:
            for name in ('subscriber','telemetry_subscriber','trace_subscriber','pong_subscriber',
                         'test_subscriber','test_publisher','probe_publisher','publisher'):
                entity = getattr(self,name,None)
                if entity is not None:
                    entity.undeclare()
        finally:
            self.transport.release(self.device)
            if self.owns_transport:
                self.transport.close()
