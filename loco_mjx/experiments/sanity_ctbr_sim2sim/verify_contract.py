import argparse
import json
from pathlib import Path
import sys

from policy_worker import PolicyWorker, ROOT
import jax
import jax.numpy as jnp
from jax.scipy.spatial.transform import Rotation as JaxRotation
import mujoco
from mujoco import mjx
import numpy as np
from scipy.spatial.transform import Rotation
from loco_mjx.environments.locomotion.urma2.mjx.default_config import get_config
from loco_mjx.environments.locomotion.urma2.mjx.environment import URMA2LocomotionEnv


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Independent native/MJX observation, symmetry and causal-history contract check")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    worker = PolicyWorker(args.checkpoint.resolve())
    cfg = get_config("locomotion.urma2.mjx")
    cfg.gait_clock_observation = worker.env.env_config.gait_clock_observation
    cfg.domain_randomization.observation_noise.type = "none"
    cfg.domain_randomization.action_delay.type = "none"
    cfg.domain_randomization.body_mixing.enabled = False
    cfg.terrain.type = "plane"
    env = URMA2LocomotionEnv(worker.env.robot_config, "test", False, cfg, 1, -1, -1)
    state = env.make_reset_state(jax.random.split(jax.random.PRNGKey(0), 1), False)
    state = jax.tree.map(lambda a: a[0], state)
    model = env.initial_mjx_model
    forward = jax.jit(mjx.forward)
    compare = jax.jit(env.get_observation)
    for a, b in zip(worker.env.get_reflection_metadata(), env.get_reflection_metadata()):
        np.testing.assert_array_equal(a, b)
    for a, b in zip(worker.env.get_body_history_metadata(), env.get_body_history_metadata()):
        np.testing.assert_array_equal(a, b)
    rng = np.random.default_rng(2718)
    maximum = 0.
    for index in range(24):
        rotation = Rotation.from_euler("xyz", rng.uniform(-.4, .4, 3))
        root = np.r_[rng.uniform(-.2, .2, 2), 1 + rng.uniform(-.2, .2), rotation.as_quat(scalar_first=True), rng.uniform(-.3, .3, 6)]
        goal = rng.uniform(-.2, .2, 4)
        action = rng.uniform(-.5, .5, 4)
        worker.set_state(root)
        worker.env.internal_state["goal_velocities"] = goal
        native = worker.env.get_observation(action, np.zeros(worker.env.nr_feet), np.zeros((worker.env.nr_feet, 4)))
        data = forward(model, state.data.replace(qpos=jnp.asarray(worker.data.qpos), qvel=jnp.asarray(worker.data.qvel)))
        internal = dict(state.internal_state)
        imu_rotation = JaxRotation.from_matrix(data.site_xmat[env.imu_site_id].reshape(3, 3))
        internal.update(goal_velocities=jnp.asarray(goal), imu_orientation_rotation=imu_rotation,
                        imu_orientation_rotation_inverse=imu_rotation.inv(), imu_orientation_euler=imu_rotation.as_euler("xyz"))
        actual = np.asarray(compare(data, model, internal, jax.random.PRNGKey(index), jnp.asarray(action), jnp.zeros(env.nr_feet), jnp.zeros((env.nr_feet, 4))))
        indices = np.r_[np.arange(4 * 55), worker.env.policy_general_obs_idx]
        error = float(np.max(np.abs(native[indices] - actual[indices])))
        maximum = max(maximum, error)
        np.testing.assert_allclose(native[indices], actual[indices], atol=3e-6, rtol=3e-6)
        imu_velocity = rotation.inv().apply(root[7:10]) + np.cross(rotation.inv().apply(root[10:13]), worker.info["imu_position"])
        np.testing.assert_allclose(worker.data.sensordata[3:6], imu_velocity, atol=1e-9)
    import importlib.util
    spec = importlib.util.spec_from_file_location("official_deployment_policy", ROOT / "deployment/unitree_go2/internal/policy.py")
    official = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(official)
    deployment_actor = official.get_policy(dict(worker.cfg))
    deployment_apply = jax.jit(deployment_actor.apply)
    deployment_error = 0.
    worker.reset()
    for index in range(12):
        rotation = Rotation.from_euler("xyz", rng.uniform(-.3, .3, 3))
        root = np.r_[0., 0., 1., rotation.as_quat(scalar_first=True), rng.uniform(-.2, .2, 6)]
        goal = rng.uniform(-.2, .2, 4)
        result = worker.infer(root, goal, commit=False)
        obs = worker.env.get_observation(worker.previous_action, np.zeros(worker.env.nr_feet), np.zeros((worker.env.nr_feet, 4)))
        obs = jnp.asarray(obs).astype(jnp.float16).astype(jnp.float32)
        joints = obs[:220].reshape(4,55)
        body = jnp.concatenate([obs[worker.body_indices], official.history_features(worker.body_memory, worker.age, worker.cfg.history_lags)])
        reference, unused = deployment_apply(worker.params, joints[:,:51], joints[:,51:], obs[worker.env.policy_general_obs_idx],
                                             reflection=worker.reflection, history_features=official.history_features(worker.joint_memory, worker.age, worker.cfg.history_lags),
                                             body_features=body, body_reflection=worker.body_reflection)
        error = float(np.max(np.abs(np.asarray(reference) - np.asarray(result["action"]))))
        deployment_error = max(deployment_error, error)
        np.testing.assert_allclose(reference, result["action"], atol=3e-6, rtol=3e-6)
        if worker.env.env_config.gait_clock_observation:
            np.testing.assert_array_equal(obs[worker.env.policy_general_obs_idx][-2:],0.)
        worker.commit_action(action=result["action"])
    worker.reset()
    initial = [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    first = worker.infer(initial, [0, 0, 0, 0])["action"]
    for index in range(12):
        worker.infer(initial, [.1, 0, 0, 0])
    worker.reset()
    repeated = worker.infer(initial, [0, 0, 0, 0])["action"]
    np.testing.assert_array_equal(first, repeated)
    history = np.asarray(worker.joint_memory)
    np.testing.assert_allclose(history[:, 0, 2], np.asarray(first) / 10, atol=1e-5)
    np.testing.assert_array_equal(np.asarray(worker.policy.history_features(worker.joint_memory, worker.age, worker.cfg.history_lags))[:, -4:], np.tile([1, 0, 0, 0], (4, 1)))
    bad_state = initial.copy()
    bad_state[7] = float("nan")
    try:
        worker.infer(bad_state, [0, 0, 0, 0])
    except ValueError:
        pass
    else:
        raise AssertionError("Non-finite input was accepted")
    disabled = worker.wrench(initial, [1, 1, 1, 1], enabled=False)
    np.testing.assert_allclose(disabled["force"] + disabled["torque"], 0, atol=1e-12)
    result = {"status": "passed", "checkpoint_sha256": worker.info["sha256"], "random_states": 24,
              "actor_observation_max_abs_error_native_vs_mjx": maximum, "reflection_metadata": "identical",
              "body_history_metadata": "identical", "imu_offset_velocity": "passed", "history_reset": "exact",
              "history_previous_action_and_validity": "passed", "nonfinite_state_rejected": True, "disabled_actuation": "zero",
              "official_deployment_actor_max_abs_error": deployment_error, "general_inputs": len(worker.env.policy_general_obs_idx),
              "scope": "Independent MJX observation construction plus official deployment actor equivalence; nominal/no-noise inputs, not a physics rollout."}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result), flush=True)
