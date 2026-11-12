"""Pinned outer-controller contract and selected-command history adapter."""

import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile
import numpy as np

PREVIEW_TIMES = (0., .1, .2, .4, .8, 1.2)
INNER_SHA256 = "25c6c781383491a5c4076320137bb6cda7882a4755450ca4242a47da4f3a7652"
CANDIDATE_SHA256 = "9688e1d811a9771d7b70b80cead51c38a4278ae89533a3a4d484f9b5ed748bcd"
CALIBRATION_SHA256 = "fb8a58c006fa65918514662a2edc742b2b6f4735f875ec0db956f84442b3abb8"


def residual_metadata(model, inner, source):
    model, inner, source = Path(model).resolve(), Path(inner).resolve(), Path(source).resolve()
    with zipfile.ZipFile(model) as archive:
        manifest = json.loads(archive.read("manifest.json"))
        payload = archive.read("parameters.msgpack")
    if manifest.get("format") != "sanity-velocity-residual-v1" or tuple(manifest.get("preview_times_s", ())) != PREVIEW_TIMES:
        raise ValueError("Residual format/trajectory preview contract mismatch")
    inner_sha = hashlib.sha256(inner.read_bytes()).hexdigest()
    if inner_sha != INNER_SHA256 or inner_sha != manifest.get("inner_sha256"):
        raise ValueError("Residual requires its exact frozen recalibrated inner checkpoint")
    if hashlib.sha256(payload).hexdigest() != manifest.get("parameters_sha256"):
        raise ValueError("Residual parameter checksum mismatch")
    if manifest.get("command_limit_m_s") != 3. or manifest.get("correction_bound_per_axis_m_s") != .5 or manifest.get("configuration", {}).get("dt_s") != .02:
        raise ValueError("Unsupported residual rate or command bounds")
    paths = [source/"loco_mjx/controllers/sanity_residual/controller.py", source/"experiments/sanity_ctbr_sim2sim/policy_worker.py"]
    hashes = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    return {"checkpoint":str(model), "sha256":hashlib.sha256(model.read_bytes()).hexdigest(),
        "inner_checkpoint":str(inner), "inner_sha256":inner_sha, "source":str(source), "source_sha256":hashes,
        "manifest":manifest, "root_state":True, "velocity_history":"PD reference feedforward; authority-weighted during CTBR blend; selected outer command at full policy; zero when output disabled",
        "vertical_command_limit_m_s":.2, "yaw_rate_limit_rad_s":.6}


def trajectory_preview(reference, landing, fallback, time_s, origin_xy, times_s=PREVIEW_TIMES):
    samples = []
    for delay in times_s:
        if landing is not None:
            ref = landing.sample(time_s+delay)
        elif fallback is not None:
            ref = {"position":fallback, "velocity":[0.]*3, "acceleration":[0.]*3}
        else:
            ref = reference.sample(time_s+delay)
            ref = {**ref, "position":(np.asarray(ref["position"])+np.r_[origin_xy,0.]).tolist()}
        samples.append([ref[key] for key in ("position","velocity","acceleration")])
    return samples


class ResidualRuntime:
    def __init__(self, model, inner, source):
        self.info = residual_metadata(model, inner, source)
        path = Path(source)/"loco_mjx/controllers/sanity_residual/controller.py"
        spec = importlib.util.spec_from_file_location("sanity_frozen_residual", path)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.outer = self.module.ResidualController(model, inner)
        self.reset()


    def reset(self):
        self.outer.reset()
        self.ticks = 0


    def propose(self, state, preview, age, enabled):
        if not np.isfinite(age) or not 0 <= age < .04:
            raise ValueError("Fresh source observation required by residual (age <40ms)")
        before = np.asarray(self.outer.hidden).tolist()
        previous_velocity = self.outer.previous_velocity.tolist()
        previous_ctbr = self.outer.previous_ctbr.tolist()
        velocity = self.outer.propose(state, preview, age, enabled=enabled)
        if not np.isfinite(velocity).all() or np.linalg.norm(velocity[:3])>3.+1e-6 or abs(velocity[2])>.2+1e-6 or abs(velocity[3])>.6+1e-6:
            raise ValueError("Residual returned an invalid velocity command")
        state_features, future = self.module.features(np.asarray(state),np.asarray(preview),np.asarray(previous_velocity),np.asarray(previous_ctbr),age)
        baseline = self.module.velocity_command(np.asarray(state),np.asarray(preview)[0],np.zeros(2))
        diagnostics = {"tick":self.ticks,"enabled":bool(enabled),"state_features":state_features.tolist(),"preview_features":future.tolist(),
            "hidden_before":before,"hidden_after":np.asarray(self.outer.hidden).tolist(),"previous_velocity_world":previous_velocity,
            "previous_selected_ctbr":previous_ctbr,"velocity_world":velocity.tolist(),"baseline_velocity_world":baseline.tolist(),
            "applied_velocity_difference_world":(velocity-baseline).tolist(),"state_age_s":age}
        return velocity, diagnostics


    def commit(self, velocity, ctbr):
        self.outer.commit(velocity, ctbr)
        self.ticks += 1
        return {"ticks":self.ticks,"selected_velocity_world":self.outer.previous_velocity.tolist(),"selected_ctbr":self.outer.previous_ctbr.tolist()}
