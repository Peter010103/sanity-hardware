"""Isolated MPC solver using the existing ordered observation/selection IPC."""

import argparse
import contextlib
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker-source', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    args = parser.parse_args()
    with contextlib.redirect_stdout(sys.stderr):
        spec = importlib.util.spec_from_file_location('sanity_ctbr_mpc', args.worker_source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        controller = module.CtbrMPC(args.model, json.loads(args.config.read_text()))
        state = [0., 0., 1., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        preview = [[[0., 0., 1.], [0., 0., 0.], [0., 0., 0.]]] * (controller.nodes+1)
        warmup = []
        for i in range(6):
            controller.command(state, preview,rate_state=module.np.zeros(6) if controller.rate_enabled else None)
            warmup.append(controller.last['solve_ms'])
        if not controller.last['success']:
            raise RuntimeError('MPC warmup failed: ' + controller.last['status'])
        controller.previous_controls = None
    info = {**controller.info, 'config_path': str(args.config.resolve()),
        'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
        'source_path': str(args.worker_source.resolve()),
        'source_sha256': hashlib.sha256(args.worker_source.read_bytes()).hexdigest(),
        'warmup_ms': warmup, 'history_contract': 'No actor history; every selected outgoing CTBR is acknowledged and logged'}
    print(json.dumps(info), flush=True)
    observer=controller.make_disturbance_observer() if controller.disturbance_enabled else None
    prediction=None
    if controller.rate_enabled:
        from sanity_zenoh.command_prediction import CommandPrediction
        prediction=CommandPrediction(controller.rate_configuration,controller.mass,controller.thrust_effectiveness,controller.drag,controller.command_delay_s)
    else:
        from sanity_zenoh.ideal_command_prediction import IdealCommandPrediction
        prediction=IdealCommandPrediction(controller.mass,controller.thrust_effectiveness,controller.drag,controller.command_delay_s)
    ticks = 0
    for line in sys.stdin:
        request = json.loads(line)
        if request['op'] == 'close':
            break
        if request['op'] == 'reset':
            controller.previous_controls = None
            controller.solver.reset()
            if observer:observer.reset()
            if prediction:prediction.reset()
            ticks = 0
            result = {'reset':True}
        elif request['op'] == 'infer':
            if request.get('test_delay_s'):
                time.sleep(request['test_delay_s'])
            computation_started=time.perf_counter()
            bias=observer.observe(request['state'],request.get('state_stamp_ns'),request.get('disturbance_context')) if observer else None
            try:
                state_for_solve=request['state'];rate_state=None
                if prediction:
                    context=request['prediction_context']
                    state_for_solve,rate_state=prediction.predict(request['state'],request['state_stamp_ns'],context['target_stamp_ns'])
                command = controller.command(state_for_solve, request['reference_preview'],disturbance_world=bias,rate_state=rate_state)
            except module.MpcSolveError:
                command = module.np.asarray(controller.last['requested_ctbr'])
            if prediction:
                controller.last['command_prediction']=prediction.last.copy()
                controller.last['total_ms']=(time.perf_counter()-computation_started)*1000
                controller.last['deadline_scope']='Causal state projection plus full MPC computation'
                if controller.last['total_ms']>controller.max_solve_time*1000:controller.last['success']=False;controller.previous_controls=None
            if observer:
                observer_ms=(time.perf_counter()-computation_started)*1000-controller.last['total_ms']
                controller.last.update(disturbance_observer=observer.last.copy(),observer_ms=max(0.,observer_ms))
                controller.last['total_ms']=(time.perf_counter()-computation_started)*1000
                controller.last['deadline_scope']='Observer plus full MPC computation; preparation, transfers and validation included'
                if controller.last['total_ms']>controller.max_solve_time*1000:
                    controller.last['success']=False
                    controller.previous_controls=None
            result = {'ctbr': command.tolist(), 'history_age': 0, 'valid': controller.last['success'],
                'mpc': controller.last, 'controller_inputs': {'state': request['state'], 'reference_preview': request['reference_preview']}}
            if observer:result['controller_inputs'].update(disturbance_world_m_s2=bias.tolist(),state_stamp_ns=request.get('state_stamp_ns'),disturbance_context=request.get('disturbance_context'))
        elif request['op'] == 'commit':
            command = module.np.asarray(request['ctbr'], dtype=float)
            if command.shape != (4,) or not module.np.isfinite(command).all():
                raise ValueError('Finite selected outgoing CTBR required')
            if prediction:prediction.commit(request['controller_context']['selected_stamp_ns'],command)
            if observer:
                context=request.get('controller_context',{})
                observer.commit(context.get('selected_stamp_ns'),command,context.get('disturbance_learning_allowed',False))
            ticks += 1
            result = {'ctbr': command.tolist(), 'ticks': ticks}
        else:
            raise ValueError('Unsupported MPC operation: ' + request['op'])
        print(json.dumps(result, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
