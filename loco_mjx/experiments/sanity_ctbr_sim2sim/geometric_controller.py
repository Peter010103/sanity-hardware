import numpy as np


def velocity_to_ctbr(goal, rotation, imu_velocity_world, mass, gravity=9.81, velocity_time_constant=.25, attitude_gain=10.):
    goal = np.asarray(goal)
    acceleration = (rotation.apply(goal[:3]) - np.asarray(imu_velocity_world)) / velocity_time_constant
    specific_force = acceleration + np.array([0., 0., gravity])
    desired_z = specific_force / max(np.linalg.norm(specific_force), 1e-9)
    yaw = rotation.as_euler('xyz')[2]
    heading = np.array([np.cos(yaw), np.sin(yaw), 0.])
    desired_y = np.cross(desired_z, heading)
    desired_y /= max(np.linalg.norm(desired_y), 1e-9)
    desired_rotation = np.column_stack([np.cross(desired_y, desired_z), desired_y, desired_z])
    current_rotation = rotation.as_matrix()
    error_matrix = .5 * (desired_rotation.T @ current_rotation - current_rotation.T @ desired_rotation)
    error = np.array([error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]])
    rates = -attitude_gain * error + current_rotation.T @ np.array([0., 0., goal[3]])
    thrust = mass * np.dot(specific_force, current_rotation[:, 2])
    return np.r_[thrust, rates]
