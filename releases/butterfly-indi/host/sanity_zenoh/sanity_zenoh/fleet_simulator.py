"""Separate loopback receiver/plant/200Hz TF fixture; never a hardware interface."""
import argparse
import gc
import ipaddress
import json
import os
from pathlib import Path
import queue
import select
import signal
import subprocess
import sys
import threading
import time


def require_loopback(endpoints):
    if set(endpoints)!={'listen','telemetry_listen'}:raise ValueError('Two explicit simulator endpoints required')
    for value in endpoints.values():
        if not ipaddress.ip_address(value.split('/',1)[1].rsplit(':',1)[0]).is_loopback:
            raise ValueError('Simulator cannot use physical/external endpoints')


class FleetSimulator:
    def __init__(self,config,nodes,output):
        require_loopback(config.transport)
        self.process=None;self.closing=False
        self.log=(output/'simulator.log').open('w')
        setup={'transport':config.transport,'mocap':config.mocap,'parent_pid':os.getpid(),
            'drones':[{'drone':d,'mass_kg':nodes[d['id']].worker.info['mass_kg'],
                'calibration_id':nodes[d['id']].mapping.calibration_id} for d in config.drones]}
        path=output/'simulator_config.json';path.write_text(json.dumps(setup,indent=2)+'\n')
        try:
            self.process=subprocess.Popen([sys.executable,'-u','-m','sanity_zenoh.fleet_simulator','--config',str(path)],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
            deadline=time.monotonic()+15.
            while time.monotonic()<deadline:
                if not select.select([self.process.stdout],[],[],max(0.,deadline-time.monotonic()))[0]:break
                line=self.process.stdout.readline()
                if not line:raise RuntimeError('Simulator exited; see simulator.log')
                try:info=json.loads(line)
                except ValueError:self.log.write(line);continue
                if info.get('ready'):self.info=info;return
            raise TimeoutError('Simulator startup timed out')
        except BaseException:self.close();raise

    @property
    def error(self):
        if not self.closing and self.process and self.process.poll() is not None:
            return 'Software simulator exited: '+str(self.process.returncode)
        return None

    def inject_fault(self,kind,device,stall_s):
        self.process.stdin.write(json.dumps({'kind':kind,'device':device,'stall_s':stall_s})+'\n')
        self.process.stdin.flush()

    def close(self):
        self.closing=True
        if self.process:
            if self.process.poll() is None:
                self.process.terminate()
                try:self.process.wait(timeout=15.)
                except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=3.)
            if self.process.stdin:self.process.stdin.close()
            if self.process.stdout:self.process.stdout.close()
        self.log.close()


def worker_main(path):
    from sanity_zenoh.fleet_emulator import SimulatedReceiver
    import rclpy
    from tf2_msgs.msg import TFMessage
    setup=json.loads(Path(path).read_text());require_loopback(setup['transport'])
    rclpy.init(args=[],signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node=rclpy.create_node('sanity_fleet_simulator');receivers={};pending=queue.SimpleQueue();stop=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
    def commands():
        for line in sys.stdin:pending.put(json.loads(line))
        stop.set()
    threading.Thread(target=commands,daemon=True).start()
    try:
        for item in setup['drones']:
            d=item['drone']
            receivers[d['id']]=SimulatedReceiver(d,setup['transport'],item['calibration_id'],item['mass_kg'],None,node,setup['mocap'])
        publisher=node.create_publisher(TFMessage,setup['mocap']['topic'],20)
        def publish():
            transforms=[receiver.transform() for receiver in receivers.values()]
            stamp=node.get_clock().now().to_msg()
            for tf in transforms:tf.header.stamp=stamp
            publisher.publish(TFMessage(transforms=transforms))
        node.create_timer(.005,publish)
        gc.collect();gc.disable()
        print(json.dumps({'ready':True,'pid':os.getpid(),'mocap_hz':200,'devices':list(receivers),'physical_endpoints':False}),flush=True)
        while rclpy.ok() and not stop.is_set() and os.getppid()==setup['parent_pid']:
            rclpy.spin_once(node,timeout_sec=.005)
            while not pending.empty():
                fault=pending.get();receiver=receivers[fault['device']]
                if fault['kind']=='receiver-restart':receiver.restart()
                elif fault['kind']=='delivery-stall':
                    with receiver.lock:receiver.hold_until=time.monotonic()+fault['stall_s']
                else:raise ValueError('Unknown software fault')
                print(json.dumps({'injected_fault':fault,'host_ns':time.monotonic_ns()}),file=sys.stderr,flush=True)
    finally:
        for receiver in receivers.values():receiver.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        gc.enable()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',required=True)
    worker_main(parser.parse_args().config)
