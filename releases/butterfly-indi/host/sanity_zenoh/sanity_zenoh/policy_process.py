"""Bounded local inference process; collect startup cycles before entering control."""

import argparse
import contextlib
import gc
import importlib.util
import json
import os
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker-source", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--residual-model", type=Path)
    parser.add_argument("--residual-source", type=Path)
    args = parser.parse_args()
    with contextlib.redirect_stdout(sys.stderr):
        spec = importlib.util.spec_from_file_location("sanity_checkpoint_worker", args.worker_source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        worker = module.PolicyWorker(args.checkpoint)
        outer = None
        if args.residual_model:
            from residual_runtime import ResidualRuntime
            from scipy.spatial.transform import Rotation
            outer = ResidualRuntime(args.residual_model,args.checkpoint,args.residual_source)
            warm_state = [0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
            warm_preview = [[[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]]]*6
            timings = []
            for _ in range(12):
                started = time.perf_counter()
                velocity,_ = outer.propose(warm_state,warm_preview,.005,True)
                result = worker.infer(warm_state,velocity,commit=False,goal_frame="body")
                worker.commit_action(ctbr=result["ctbr"])
                outer.commit(velocity,result["ctbr"])
                timings.append((time.perf_counter()-started)*1000)
            outer.reset()
            worker.reset()
            worker.info["residual"] = {**outer.info,"warmup_ms":timings,"warmup_complete":True}
        gc.collect()
        gc.disable()
    info = {**worker.info, "runtime": {"cyclic_gc_enabled": gc.isenabled(), "pid": os.getpid(),
                                      "scope": "Parent owns mission runtime; process exits after each session"}}
    print(json.dumps(info), flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        op = request["op"]
        if op == "close":
            break
        if op == "reset":
            worker.reset()
            if outer is not None:
                outer.reset()
            result = {"reset": True}
        elif op == "ground_fixture":
            result = worker.ground_fixture()
            gc.collect()
        elif op == "advance":
            worker.set_state(request["state"])
            worker.data.ctrl[:] = request["ctbr"]
            module.mujoco.mj_step(worker.model, worker.data, nstep=request.get("substeps", 1))
            module.mujoco.mj_forward(worker.model, worker.data)
            result = {"state": worker.root_state(), "contact_count": int(worker.data.ncon)}
        elif op == "infer":
            if request.get("test_delay_s",0):
                time.sleep(request["test_delay_s"])
            started = time.perf_counter()
            outer_diagnostics = None
            if outer is not None:
                if request.get("commit",True):
                    raise ValueError("Residual needs an explicit selected-command commit")
                world_velocity,outer_diagnostics = outer.propose(request["state"],request["preview"],request["state_age_s"],request["residual_enabled"])
                rotation = Rotation.from_quat(module.np.asarray(request["state"])[[4,5,6,3]])
                request["goal"] = [*rotation.inv().apply(world_velocity[:3]),float(world_velocity[3])]
                request["goal_frame"] = "body"
            original = worker.apply
            if request.get("record_inputs",False):
                def capture(*values):
                    recorded[:] = [module.np.asarray(v).tolist() for v in values]
                    return original(*values)
                recorded = []
                worker.apply = capture
            try:
                result = worker.infer(request["state"], request["goal"], commit=request.get("commit", True), goal_frame=request.get("goal_frame", "body"))
            finally:
                worker.apply = original
            if request.get("record_inputs",False):
                result["actor_inputs"] = dict(zip(["descriptors","joint_state","general_state","joint_history","body_history"],recorded))
            if outer_diagnostics is not None:
                result.update(outer=outer_diagnostics,goal=request["goal"],velocity_world=world_velocity.tolist(),combined_inference_ms=(time.perf_counter()-started)*1000)
        elif op == "commit":
            result = worker.commit_action(action=request.get("action"), ctbr=request.get("ctbr"))
            if outer is not None:
                result["outer"] = outer.commit(request["selected_velocity_world"],request["outgoing_ctbr"])
        else:
            raise ValueError(f"Unsupported live operation: {op}")
        print(json.dumps(result, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
