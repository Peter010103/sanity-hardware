#!/usr/bin/env python3
"""Prebuild and warm an isolated MPC worker; never opens a transport or ROS."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh'))
sys.path.insert(0,str(ROOT/'Sanity/src/ros2_sanity_ws/src/udp_broadcaster'))
from sanity_zenoh.async_policy import AsyncPolicy


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--model',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    worker=AsyncPolicy(Path(sys.executable),args.source,args.config,args.output,controller='mpc',mpc_model=args.model)
    try:
        (args.output/'solver_info.json').write_text(json.dumps(worker.info,indent=2)+'\n')
        print(json.dumps(dict(prepared=True,hardware_access=False,worker_pid=worker.actor.process.pid,solver=worker.info),indent=2))
    finally:worker.close()

if __name__=='__main__':main()
