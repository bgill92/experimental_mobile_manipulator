#!/usr/bin/env python3
"""Drive the mecanum base in plain MuJoCo and check it moves the commanded way.

Usage: check_mecanum.py [scene.xml]

Runs forward, strafe-left and rotate-left for 2 s each, closing a velocity loop on the wheel
motors the way mujoco_ros2_control does with config/wheel_pids.yaml. Exits non-zero on failure.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

PKG = Path(__file__).resolve().parent.parent
WHEELS = ["front_left", "front_right", "rear_left", "rear_right"]
# Must match the mecanum_drive_controller kinematics in config/controllers.yaml.
CONTROLLERS = yaml.safe_load((PKG / "config/controllers.yaml").read_text())
KIN = CONTROLLERS["mecanum_drive_controller"]["ros__parameters"]["kinematics"]
PIDS = yaml.safe_load((PKG / "config/wheel_pids.yaml").read_text())
GAINS = PIDS["/**"]["ros__parameters"]["pid_gains"]["velocity"]


def wheel_speeds(vx: float, vy: float, wz: float) -> np.ndarray:
    # Standard X-layout mecanum IK, same as mecanum_drive_controller.
    k = KIN["sum_of_robot_center_projection_on_X_Y_axis"]
    return np.array([vx - vy - k * wz, vx + vy + k * wz, vx + vy - k * wz, vx - vy + k * wz]) / KIN["wheels_radius"]


def drive(model, data, twist, seconds=2.0):
    joints = [model.joint(f"{w}_wheel_joint") for w in WHEELS]
    acts = [model.actuator(f"{w}_wheel_joint").id for w in WHEELS]
    target = wheel_speeds(*twist)
    start = data.qpos[:7].copy()
    # The plugin updates the PIDs once per controller cycle and holds ctrl between sim steps.
    hold = round(1 / CONTROLLERS["controller_manager"]["ros__parameters"]["update_rate"] / model.opt.timestep)
    for step in range(int(seconds / model.opt.timestep)):
        if step % hold == 0:
            for j, a, t, w in zip(joints, acts, target, WHEELS):
                g = GAINS[f"{w}_wheel_joint"]
                data.ctrl[a] = np.clip(g["p"] * (t - data.qvel[j.dofadr[0]]), g["u_clamp_min"], g["u_clamp_max"])
        mujoco.mj_step(model, data)
    # Displacement in the start heading's frame, plus yaw change.
    yaw = lambda q: np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    y0 = yaw(start[3:7])
    d = data.qpos[:2] - start[:2]
    local = np.array([np.cos(y0) * d[0] + np.sin(y0) * d[1], -np.sin(y0) * d[0] + np.cos(y0) * d[1]])
    return local[0], local[1], yaw(data.qpos[3:7]) - y0


def main() -> None:
    scene = sys.argv[1] if len(sys.argv) > 1 else str(PKG / "mujoco/scene.xml")
    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    drive(model, data, (0, 0, 0), 1.0)  # Settle onto the rollers.

    ok = True
    # Commanded 0.1 m/s or 0.5 rad/s for 2 s: expect at least half of the ideal 0.2 m / 1 rad,
    # and cross-axis drift under a quarter of it.
    for name, twist, axis in [("forward", (0.1, 0, 0), 0), ("strafe left", (0, 0.1, 0), 1),
                              ("rotate left", (0, 0, 0.5), 2)]:
        moved = np.array(drive(model, data, twist))
        ideal = 2 * max(twist)
        drift = np.delete(np.abs(moved[:2]), axis) if axis < 2 else np.abs(moved[:2])
        passed = moved[axis] > ideal / 2 and np.all(drift < ideal / 4 if axis < 2 else drift < 0.05)
        ok &= passed
        print(f"{'PASS' if passed else 'FAIL'} {name}: dx={moved[0]:.3f} m dy={moved[1]:.3f} m dyaw={moved[2]:.3f} rad")
        drive(model, data, (0, 0, 0), 1.0)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
