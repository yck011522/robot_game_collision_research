"""Standalone PyBullet collision-check benchmark.

Loads a serialized compas_fab RobotCell + RobotCellState from
`robot_cell_and_state.json`, pushes it into a PyBullet GUI session via the
compas_fab PyBulletClient, exposes 6 joint sliders + an "auto move" checkbox,
runs collision checks every frame, and prints/overlays the current FPS.

Run from the repo root (or anywhere) with the `game` conda env active:

    conda activate game
    python pybullet/bullet_collision.py
"""

from __future__ import annotations

import math
import os
import time
from collections import deque

# NOTE: import compas_fab's PyBullet client BEFORE pybullet itself.
# `compas_fab.backends.pybullet.client` only binds its module-global
# `pybullet` name through a LazyLoader when pybullet hasn't been imported
# yet; otherwise its `connect()` raises NameError. (compas_fab 1.1.0 bug.)
from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.backends.exceptions import CollisionCheckError

import pybullet as p  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(HERE, "robot_cell_and_state.json")

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def main():
    data = json_load(JSON_PATH)
    robot_cell = data["robot_cell"]
    robot_cell_state = data["robot_cell_state"]

    # Workaround: serialized URDF <transmission> elements come back as
    # tag-less URDFGenericElement objects and crash URDF export inside
    # PyBulletClient. Transmissions are irrelevant for collision checking.
    robot_cell.robot_model.attr.pop("transmission", None)

    # Add allowed touch to the following special case:
    # - The robot's robot link 'base_link_inertia' and rigid body 'RB8' (body_id '9')
    robot_cell_state.rigid_body_states["RB8"].touch_links = ["base_link_inertia"]

    joints = {j.name: j for j in robot_cell.robot_model.get_configurable_joints()}
    lower = [
        joints[n].limit.lower if joints[n].limit else -math.pi for n in JOINT_NAMES
    ]
    upper = [joints[n].limit.upper if joints[n].limit else math.pi for n in JOINT_NAMES]

    start_cfg = robot_cell_state.robot_configuration
    start_vals = list(start_cfg.joint_values)

    with PyBulletClient(connection_type="gui", verbose=False) as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(robot_cell)
        planner.set_robot_cell_state(robot_cell_state)

        # Nicer camera + minimal GUI chrome.
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=[0, 0, 0.3],
        )

        # Sliders for the 6 joints + auto-move toggle + speed.
        slider_ids = [
            p.addUserDebugParameter(name, lower[i], upper[i], start_vals[i])
            for i, name in enumerate(JOINT_NAMES)
        ]
        auto_id = p.addUserDebugParameter("auto move (>=0.5 on)", 0.0, 1.0, 0.0)
        speed_id = p.addUserDebugParameter("auto speed (rad/s)", 0.0, 3.0, 0.6)

        # Overlay text handle (re-used).
        text_id = None

        fps_window = deque(maxlen=60)
        cc_ms_window = deque(maxlen=60)
        last_t = time.perf_counter()
        t_start = last_t
        last_report = last_t

        cfg = start_cfg.copy()

        while p.isConnected():
            now = time.perf_counter()
            dt = now - last_t
            last_t = now
            fps_window.append(dt)

            auto_on = p.readUserDebugParameter(auto_id) >= 0.5
            speed = p.readUserDebugParameter(speed_id)

            if auto_on:
                # Smooth sinusoidal sweep within each joint's limits.
                t = now - t_start
                vals = []
                for i in range(6):
                    mid = 0.5 * (lower[i] + upper[i])
                    amp = 0.45 * (upper[i] - lower[i])
                    # Different phase per joint so they don't all move together.
                    vals.append(mid + amp * math.sin(speed * t + i * 0.9))
                cfg.joint_values = vals
            else:
                vals = [p.readUserDebugParameter(sid) for sid in slider_ids]
                cfg.joint_values = vals

            robot_cell_state.robot_configuration = cfg

            # Collision check + timing.
            t0 = time.perf_counter()
            in_collision = False
            collision_msg = ""
            try:
                planner.check_collision(robot_cell_state)
            except CollisionCheckError as e:
                in_collision = True
                collision_msg = str(e).splitlines()[0] if str(e) else "collision"
            cc_ms = (time.perf_counter() - t0) * 1000.0
            cc_ms_window.append(cc_ms)

            # Periodic overlay update (every ~0.1 s, not every frame, to avoid
            # spamming the GUI with debug text replacements).
            if now - last_report > 0.1:
                last_report = now
                avg_dt = sum(fps_window) / len(fps_window)
                fps = 1.0 / avg_dt if avg_dt > 0 else 0.0
                avg_cc = sum(cc_ms_window) / len(cc_ms_window)
                status = "COLLISION" if in_collision else "free"
                text = "FPS {:5.1f}  |  CC {:5.2f} ms ({:6.0f} Hz)  |  {}".format(
                    fps, avg_cc, 1000.0 / avg_cc if avg_cc > 0 else 0.0, status
                )
                color = [1, 0.2, 0.2] if in_collision else [0.2, 1, 0.2]
                kwargs = dict(
                    textPosition=[-0.5, -0.5, 1.2],
                    textColorRGB=color,
                    textSize=1.3,
                )
                if text_id is not None:
                    kwargs["replaceItemUniqueId"] = text_id
                text_id = p.addUserDebugText(text, **kwargs)
                if in_collision:
                    print(
                        "[{:6.1f} Hz] {}".format(
                            1000.0 / avg_cc if avg_cc else 0, collision_msg
                        )
                    )


if __name__ == "__main__":
    main()
