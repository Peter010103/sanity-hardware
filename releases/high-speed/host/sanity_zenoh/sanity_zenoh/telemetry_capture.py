"""Read-only Zenoh telemetry recording; never publishes aircraft commands."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
from pathlib import Path
import signal
import time

from sanity_zenoh.fc_telemetry import decode_fc_telemetry


def summarize_telemetry(events):
    counts=Counter();samples=defaultdict(list);health=[];errors=0;dropped=0;outputs=[];rpm=[];output_stamps=defaultdict(list)
    for event in events:
        if event.get("kind")!="zenoh_rx_fc_telemetry":continue
        if "decode_error" in event:
            errors+=1;continue
        telemetry=event.get("telemetry")
        if telemetry is None:
            try:telemetry=decode_fc_telemetry(bytes.fromhex(event["payload_hex"]))
            except (ValueError,KeyError):errors+=1;continue
        dropped=max(dropped,telemetry["esp_dropped_records"])
        for record in telemetry["records"]:
            counts[str(record["type"])+":"+str(record["msp_command"])]+=1
            if "decode_error" in record:errors+=1
            if record["type"]==2:
                samples[(record["epoch"],record["msp_command"])].append(record["response_parsed_esp_us"])
            if "esp_health" in record:health.append(record["esp_health"])
            if record["type"]==1 and record["msp_command"]==200:
                outputs.append(record["uart_write_return_esp_us"]-record["uart_write_begin_esp_us"])
                output_stamps[record["epoch"]].append(record["uart_write_begin_esp_us"])
            if "motors" in record:
                rpm.append(all(m["rpm_quality_usable"] for m in record["motors"]) and len(record["motors"])==4)
    rates={}
    for command in (102,104,105,130,139,200):
        gaps=[]
        for (epoch,c),stamps in samples.items():
            if c==command:gaps.extend(b-a for a,b in zip(stamps,stamps[1:]) if b>a)
        rates[str(command)]={"received_samples":sum(count for key,count in counts.items() if key=="2:"+str(command)),
            "mean_rate_hz":1e6*len(gaps)/sum(gaps) if gaps else None,"maximum_observed_gap_ms":max(gaps)/1000 if gaps else None}
    output_gaps=[b-a for stamps in output_stamps.values() for a,b in zip(stamps,stamps[1:]) if b>a]
    return {"telemetry_received":bool(counts),"record_counts":dict(counts),"decode_errors":errors,"maximum_reported_esp_dropped_records":dropped,
        "response_timing":rates,"maximum_rc_uart_write_call_duration_us":max(outputs) if outputs else None,
        "rc_output_mean_rate_hz":1e6*len(output_gaps)/sum(output_gaps) if output_gaps else None,
        "maximum_observed_rc_output_gap_ms":max(output_gaps)/1000 if output_gaps else None,
        "rpm_samples_with_all_four_quality_usable":sum(rpm),"rpm_samples":len(rpm),
        "esp_boots":list({h["boot_id"]:{"boot_id":h["boot_id"],"boot_count":h["boot_count"],"reset_reason":h["reset_reason"]} for h in health}.values()),
        "latest_esp_health":health[-1] if health else None,
        "timing_scope":"Rates/gaps use ESP parse timestamps. UART write return means queued, not FC actuation. No one-way network latency inferred.",
        "command_output":"No commands are published by telemetry_capture"}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen",default="udp/10.0.0.200:7447")
    parser.add_argument("--telemetry-listen",default="tcp/10.0.0.200:7448",help="Independent bulk listener; empty string uses the control session")
    parser.add_argument("--connect")
    parser.add_argument("--device",type=int,default=9)
    parser.add_argument("--duration",type=float,default=30.)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--events",type=Path,help="Summarize existing experiment JSONL without opening a network session")
    args=parser.parse_args()
    if args.events:
        report=summarize_telemetry(json.loads(line) for line in args.events.open())
        print(json.dumps(report,indent=2));return
    if not 1<=args.device<=255 or not 0<args.duration<=3600:parser.error("device1..255; duration in(0,3600] seconds")
    output=args.output or Path(__file__).resolve().parents[6]/"outputs"/("sanity_telemetry_"+datetime.now().strftime("%Y%m%d_%H%M%S"))
    output.mkdir(parents=True,exist_ok=False)
    from sanity_zenoh.recording import EventRecorder
    from sanity_zenoh.protocol import ReceiverStatus
    from dataclasses import asdict
    import zenoh
    events=EventRecorder(output/"events.jsonl")
    def receive_fc(sample):
        raw=bytes(sample.payload);event={"kind":"zenoh_rx_fc_telemetry","payload_hex":raw.hex()}
        try:event["telemetry"]=decode_fc_telemetry(raw,args.device)
        except (ValueError,TypeError) as error:event["decode_error"]=str(error)
        events(event)
    def receive_status(sample):
        raw=bytes(sample.payload);event={"kind":"zenoh_rx_status","payload_hex":raw.hex()}
        try:event["status"]=asdict(ReceiverStatus.decode(raw))
        except (ValueError,TypeError) as error:event["decode_error"]=str(error)
        events(event)
    config=zenoh.Config()
    config.insert_json5("mode",json.dumps("client" if args.connect else "peer"))
    config.insert_json5("scouting/multicast/enabled","false");config.insert_json5("scouting/gossip/enabled","false")
    config.insert_json5("listen/endpoints",json.dumps([] if args.connect else [args.listen]))
    config.insert_json5("connect/endpoints",json.dumps([args.connect] if args.connect else []))
    session=None;telemetry_session=None;error=None;stopped=False
    def stop(signum,frame):
        nonlocal stopped
        stopped=True
    old={s:signal.signal(s,stop) for s in (signal.SIGINT,signal.SIGTERM)}
    print("Recording read-only telemetry:",output,flush=True)
    try:
        session=zenoh.open(config)
        if args.telemetry_listen:
            bulk=zenoh.Config()
            bulk.insert_json5("mode",'"peer"')
            bulk.insert_json5("scouting/multicast/enabled","false");bulk.insert_json5("scouting/gossip/enabled","false")
            bulk.insert_json5("listen/endpoints",json.dumps([args.telemetry_listen]))
            bulk.insert_json5("connect/endpoints","[]")
            telemetry_session=zenoh.open(bulk)
        root=f"sanity/{args.device:02d}"
        subscribers=[(telemetry_session or session).declare_subscriber(root+"/telemetry/fc",receive_fc),session.declare_subscriber(root+"/status",receive_status)]
        started=time.monotonic()
        while not stopped and time.monotonic()-started<args.duration:
            if events.error:raise RuntimeError(events.error)
            time.sleep(.05)
        for subscriber in subscribers:subscriber.undeclare()
    except Exception as exc:error=str(exc)
    finally:
        if telemetry_session is not None:telemetry_session.close()
        if session is not None:session.close()
        events.close()
        for sig,previous in old.items():signal.signal(sig,previous)
    report=summarize_telemetry(json.loads(line) for line in (output/"events.jsonl").open())
    report.update(error=error,recording_error=events.error)
    (output/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))
    if error or events.error or not report["telemetry_received"]:raise SystemExit(2)


if __name__=="__main__":main()
