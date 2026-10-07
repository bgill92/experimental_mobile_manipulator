"""In-process arm planning for emma on top of roboplan. Kept free of rclpy."""

from collections.abc import Sequence
import copy
from dataclasses import dataclass
import os
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Duration
from emma_manipulation.constants import ARM_JOINTS, BASE_FRAME, TCP_FRAME
import numpy as np
from numpy.typing import ArrayLike
from roboplan.core import (
    Box,
    CartesianConfiguration,
    JointConfiguration,
    loadUrdfSceneDescriptionFromXml,
    PathShortcutter,
    PathShortcuttingOptions,
    Scene,
)
from roboplan.rrt import RRT, RRTOptions
from roboplan.simple_ik import SimpleIk, SimpleIkOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

GROUP = 'arm'

# Packages whose package:// meshes the URDF references.
MESH_PACKAGES = ['emma_description', 'mycobot_description', 'myagv_description']

# coal/assimp ignore the COLLADA <unit> tag, so meshes in other units load at the wrong
# size (gripper 1000x, base 39x). RViz honours the tag, so the URDF stays as is and the
# scale is applied on the planning side only.
MESH_SCALES = {
    'parallel_gripper/': '0.001 0.001 0.001',  # Millimetres.
    'myagv_': '0.0254 0.0254 0.0254',  # Inches.
}

# Link pairs that never need checking: the fingers only slide relative to the wrist, and
# the wrist camera is bolted to the gripper next to it. Pairs naming a link the URDF lacks
# (no camera) are skipped.
DISABLED_COLLISION_PAIRS = [
    (part, wrist)
    for part in ('gripper_left', 'gripper_right', 'wrist_camera_link')
    for wrist in ('joint6', 'joint6_flange')
]

# roboplan places added geometry relative to the parent frame's *joint*, not the frame
# itself, so held objects hang off the link of the last arm joint (whose frame is that
# joint's frame) rather than off tcp. base_link coincides with the root, so obstacles in
# base_link are unaffected.
ATTACH_LINK = 'joint6_flange'

# Links that may touch an object held in the gripper.
GRIPPER_LINKS = ['gripper_base', 'gripper_left', 'gripper_right']


def load_urdf_for_planning(urdf_xml: str) -> tuple[str, list[str]]:
    """
    Adapt emma's URDF for roboplan and return it with the package search paths.

    - Mesh scales fix the unit mismatch described at MESH_SCALES.
    - The myAGV links have visuals only; their meshes are copied into collisions so the
      planner keeps the arm out of the base.
    - <mimic> tags are dropped: Pinocchio requires the mimicked joint to come first and
      here it does not. The right finger becomes an independent joint left at 0 (open).
    """
    from ament_index_python.packages import get_package_share_directory

    root = ET.fromstring(urdf_xml)
    for joint in root.iter('joint'):
        for mimic in joint.findall('mimic'):
            joint.remove(mimic)
    for link in root.iter('link'):
        visual = link.find('visual')
        if visual is None or link.find('collision') is not None:
            continue
        mesh = visual.find('geometry/mesh')
        if mesh is None or 'myagv_' not in mesh.get('filename', ''):
            continue
        collision = ET.SubElement(link, 'collision')
        for child in visual:
            if child.tag in ('origin', 'geometry'):
                collision.append(copy.deepcopy(child))
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename', '')
        for key, scale in MESH_SCALES.items():
            if key in filename:
                mesh.set('scale', scale)

    # package://pkg/... resolves against the directory holding the package share dir.
    package_paths = [
        os.path.dirname(get_package_share_directory(pkg)) for pkg in MESH_PACKAGES
    ]
    return ET.tostring(root, encoding='unicode'), package_paths


@dataclass
class _Box:
    size: tuple[float, float, float]
    rgba: np.ndarray
    frame: str
    tform: np.ndarray  # Box pose in `frame`.


class ArmPlanner:
    """
    Collision-aware IK and motion planning for the arm, in joint space of ARM_JOINTS.

    Configurations `q` are 6-vectors in ARM_JOINTS order. Poses are 4x4 transforms of
    `tip_frame` in `base_frame`. The base, wheels and gripper fingers stay at their
    default positions (fingers open) for collision checking.

    @warning Not thread-safe: the scene is shared and mutated by add/attach calls.
    """

    def __init__(
        self,
        urdf_xml: str,
        base_frame: str = BASE_FRAME,
        tip_frame: str = TCP_FRAME,
        max_planning_time: float = 2.0,
        seed: int | None = None,
    ) -> None:
        self.base_frame = base_frame
        self.tip_frame = tip_frame
        urdf, package_paths = load_urdf_for_planning(urdf_xml)
        self.scene = Scene('emma', loadUrdfSceneDescriptionFromXml(urdf, package_paths))
        self.scene.addGroupFromChain(GROUP, 'g_base', tip_frame)
        self.scene.allowAdjacentLinkCollisions()
        links = {link.get('name') for link in ET.fromstring(urdf).iter('link')}
        self.scene.setCollisions(
            [pair for pair in DISABLED_COLLISION_PAIRS if set(pair) <= links], False)
        if seed is not None:
            self.scene.setRngSeed(seed)

        self._q_full = np.array(self.scene.getCurrentJointPositions())
        self._arm_idx = np.array(self.scene.getJointPositionIndices(ARM_JOINTS))
        self._boxes: dict[str, _Box] = {}

        self._ik = SimpleIk(self.scene, SimpleIkOptions(
            group_name=GROUP, max_iters=200, max_time=0.05, max_restarts=5,
            check_collisions=True))
        # Linear moves must stay on the seed's IK branch, so no random restarts.
        self._ik_local = SimpleIk(self.scene, SimpleIkOptions(
            group_name=GROUP, max_iters=200, max_time=0.05, max_restarts=0,
            check_collisions=True))
        self._rrt = RRT(self.scene, RRTOptions(
            group_name=GROUP, max_connection_distance=1.0, collision_check_step_size=0.02,
            max_planning_time=max_planning_time, rrt_connect=True))
        self._shortcutter = PathShortcutter(self.scene, PathShortcuttingOptions(
            group_name=GROUP, max_step_size=0.02, max_iters=200,
            seed=-1 if seed is None else seed))
        if seed is not None:
            self._ik.setRngSeed(seed)
            self._rrt.setRngSeed(seed)

    # Configurations.

    def set_q(self, joint_state: JointState) -> np.ndarray:
        """Pick the ARM_JOINTS positions out of a JointState, in ARM_JOINTS order."""
        by_name = dict(zip(joint_state.name, joint_state.position))
        missing = [j for j in ARM_JOINTS if j not in by_name]
        if missing:
            raise ValueError(f'JointState lacks arm joints: {missing}')
        return np.array([by_name[j] for j in ARM_JOINTS])

    def full_q(self, q: ArrayLike) -> np.ndarray:
        """Expand an arm configuration to the scene's full position vector."""
        full = self._q_full.copy()
        full[self._arm_idx] = q
        return full

    def has_collisions(self, q: ArrayLike) -> bool:
        return bool(self.scene.hasCollisions(self.full_q(q)))

    def fk(self, q: ArrayLike, frame: str | None = None) -> np.ndarray:
        """Pose of `frame` (default: the tip) in the base frame."""
        return np.array(self.scene.forwardKinematics(
            self.full_q(q), frame or self.tip_frame, self.base_frame))

    # IK and planning.

    def ik(self, tform: np.ndarray, q_seed: ArrayLike, local: bool = False
           ) -> np.ndarray | None:
        """Collision-free IK for the tip at `tform`, or None. `local` disables restarts."""
        goal = CartesianConfiguration(self.base_frame, self.tip_frame, np.asarray(tform))
        start = JointConfiguration(ARM_JOINTS, np.asarray(q_seed, dtype=float))
        solution = JointConfiguration()
        solver = self._ik_local if local else self._ik
        if not solver.solveIk(goal, start, solution):
            return None
        return np.array(solution.positions)

    def plan_joint(self, q_start: ArrayLike, q_goal: ArrayLike
                   ) -> list[np.ndarray] | None:
        """Collision-free joint path from `q_start` to `q_goal` (RRT-Connect, shortcut)."""
        if self.has_collisions(q_goal):
            return None
        start = JointConfiguration(ARM_JOINTS, np.asarray(q_start, dtype=float))
        goal = JointConfiguration(ARM_JOINTS, np.asarray(q_goal, dtype=float))
        try:
            path = self._rrt.plan(start, goal)
        except RuntimeError:
            return None
        if path is None or len(path.positions) == 0:
            return None
        path = self._shortcutter.shortcut(path)
        return [np.array(p) for p in path.positions]

    def plan_to_any(self, q_start: ArrayLike, tforms: Sequence[np.ndarray]
                    ) -> tuple[list[np.ndarray], int] | None:
        """
        Plan to the first reachable pose of `tforms`, in order of preference.

        Returns the path and the index of the pose it reaches, or None. roboplan 0.7 has
        no multi-goal RRT, so each pose is solved by IK and planned to in turn.
        """
        for index, tform in enumerate(tforms):
            q_goal = self.ik(tform, q_start)
            if q_goal is None:
                continue
            path = self.plan_joint(q_start, q_goal)
            if path is not None:
                return path, index
        return None

    def plan_linear(self, q_start: ArrayLike, tform: np.ndarray, steps: int = 10,
                    max_joint_jump: float = 1.0) -> list[np.ndarray] | None:
        """
        Short straight move: IK seeded at `q_start`, then a collision-checked joint lerp.

        Over a few centimetres the TCP stays close to a line. Fails if IK lands on another
        branch (any joint moves more than `max_joint_jump` rad) or any step collides.
        """
        q0 = np.asarray(q_start, dtype=float)
        q_goal = self.ik(tform, q0, local=True)
        if q_goal is None or np.max(np.abs(q_goal - q0)) > max_joint_jump:
            return None
        path = [q0 + (q_goal - q0) * i / steps for i in range(steps + 1)]
        if any(self.has_collisions(q) for q in path[1:]):
            return None
        return path

    # Scene objects.

    def add_box(self, name: str, size_xyz: Sequence[float], tform_in_base: np.ndarray,
                rgba: Sequence[float] = (0.6, 0.6, 0.6, 1.0)) -> None:
        """Add (or replace) a box obstacle with full side lengths `size_xyz`."""
        if name in self._boxes:
            self.remove(name)
        sx, sy, sz = (float(s) for s in size_xyz)
        box = _Box((sx, sy, sz), np.asarray(rgba, dtype=float),
                   self.base_frame, np.asarray(tform_in_base, dtype=float))
        self._add(name, box)

    def has_box(self, name: str) -> bool:
        return name in self._boxes

    def remove(self, name: str) -> None:
        self.scene.removeGeometry(name)
        del self._boxes[name]

    def attach(self, name: str, q: ArrayLike) -> None:
        """Rigidly attach box `name` to the wrist where it is at configuration `q`."""
        box = self._boxes[name]
        t_box_base = self.fk(q, box.frame) @ box.tform
        t_link = self.fk(q, ATTACH_LINK)
        self._move(name, ATTACH_LINK, np.linalg.inv(t_link) @ t_box_base)
        self.scene.setCollisions([(name, link) for link in GRIPPER_LINKS], False)

    def detach(self, name: str, q: ArrayLike) -> None:
        """Leave box `name` where it is at configuration `q`, fixed in the base frame."""
        box = self._boxes[name]
        t_box_base = self.fk(q, box.frame) @ box.tform
        self._move(name, self.base_frame, t_box_base)

    def _add(self, name: str, box: _Box) -> None:
        self.scene.addBoxGeometry(name, box.frame, Box(*box.size), box.tform, box.rgba)
        self._boxes[name] = box
        # Objects resting on each other are not collisions.
        others = [(name, other) for other in self._boxes if other != name]
        if others:
            self.scene.setCollisions(others, False)

    def _move(self, name: str, frame: str, tform: np.ndarray) -> None:
        box = self._boxes[name]
        self.scene.removeGeometry(name)
        del self._boxes[name]
        self._add(name, _Box(box.size, box.rgba, frame, tform))


def to_joint_trajectory(path: Sequence[ArrayLike], v_max: float = 0.8,
                        min_segment_time: float = 0.1) -> JointTrajectory:
    """
    Time a joint path for arm_controller with a per-segment max joint speed.

    The first waypoint is the start state and is dropped. Positions only; the
    controller interpolates. Proper time parameterisation (TOPPRA) needs a joint
    limits YAML and is deferred.
    """
    traj = JointTrajectory()
    traj.joint_names = list(ARM_JOINTS)
    t = 0.0
    points = [np.asarray(p, dtype=float) for p in path]
    for prev, point in zip(points[:-1], points[1:]):
        t += max(float(np.max(np.abs(point - prev))) / v_max, min_segment_time)
        secs, nanosecs = divmod(int(round(t * 1e9)), 10**9)
        traj.points.append(JointTrajectoryPoint(
            positions=point.tolist(),
            time_from_start=Duration(sec=secs, nanosec=nanosecs)))
    return traj
