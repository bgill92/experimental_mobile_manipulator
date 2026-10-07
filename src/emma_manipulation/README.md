# emma_manipulation

Arm motion planning for **emma** with [roboplan](https://github.com/open-planning/roboplan) 0.7,
used as an in-process Python library: collision-aware IK, RRT-Connect with path shortcutting,
short straight-line moves, and top-down grasp poses for a small block. `planner.py` and
`grasp.py` have no ROS node in them; `move_arm` is the thin ROS layer that runs a plan on the
simulated (or real) `arm_controller`.

## Contents

| Path | What it is |
|---|---|
| `emma_manipulation/constants.py` | Arm joint names, `HOME_Q`, frames, table/block geometry, grasp tilts, gripper positions. |
| `emma_manipulation/planner.py` | `ArmPlanner`, `to_joint_trajectory`, `load_urdf_for_planning`. |
| `emma_manipulation/grasp.py` | `grasp_candidates`, `offset`. Pure numpy. |
| `emma_manipulation/move_arm.py` | `move_arm` console script. |
| `test/test_planner.py` | Planner checks against the real URDF, including the reachability gate for the block. |
| `test/test_grasp.py` | Grasp pose geometry. |

## move_arm

Start the sim (`pixi run sim`), then in another terminal:

```bash
pixi run bash -c "source install/setup.bash && ros2 run emma_manipulation move_arm --pose 0.25 0 0.20 0 3.1416 0"
pixi run bash -c "source install/setup.bash && ros2 run emma_manipulation move_arm --linear 0.25 0 0.17 0 3.1416 0"
pixi run bash -c "source install/setup.bash && ros2 run emma_manipulation move_arm --home"
```

| Option | Effect |
|---|---|
| `--home` | Plan (RRT) to `HOME_Q`, the folded start pose. |
| `--joints q1 … q6` | Plan (RRT) to arm joint positions in rad, in `ARM_JOINTS` order. |
| `--pose x y z [roll pitch yaw]` | Plan (IK + RRT) to a TCP pose in `base_link`. Angles are fixed-axis x, y, z in rad. `0 3.1416 0` points the gripper straight down. |
| `--linear x y z [roll pitch yaw]` | Short straight move to a TCP pose (IK seeded at the current state, then a joint-space interpolation). |
| `--no-table` | Leave the table box out of the planning scene. |
| `--v-max` | Max joint speed for the trajectory timing, rad/s (default 0.8). |

It reads `/robot_description` and `/joint_states`, sends the trajectory to
`/arm_controller/follow_joint_trajectory`, waits for the result, and logs the reached TCP pose
and its error. In the sim the error is typically 2–5 mm: the position actuators sag a little
under gravity.

## Python API

```python
from emma_manipulation.planner import ArmPlanner, to_joint_trajectory
from emma_manipulation.grasp import grasp_candidates, offset

planner = ArmPlanner(urdf_xml)                       # base_link → tcp
q = planner.set_q(joint_state_msg)                   # 6-vector in ARM_JOINTS order
planner.add_box('table', size_xyz, tform_in_base)    # full side lengths
grasps = grasp_candidates(block_xyz, block_yaw, arm_base_xy)
path, index = planner.plan_to_any(q, [offset(g, 0.05) for g in grasps])
down = planner.plan_linear(path[-1], grasps[index])
planner.attach('block', down[-1])                    # held objects move with the wrist
trajectory = to_joint_trajectory(path)               # trajectory_msgs/JointTrajectory
```

| Call | Returns |
|---|---|
| `fk(q, frame=tcp)` | 4×4 pose of `frame` in `base_link`. |
| `ik(tform, q_seed, local=False)` | Collision-free IK solution or `None`. `local` disables random restarts. |
| `plan_joint(q_start, q_goal)` | Shortcut RRT-Connect path (list of 6-vectors, including the start) or `None`. |
| `plan_to_any(q_start, tforms)` | `(path, index)` for the first pose in `tforms` that IK and RRT reach, or `None`. |
| `plan_linear(q_start, tform, steps=10)` | Joint-interpolated path to `tform`, or `None` if IK jumps to another branch or a step collides. |
| `add_box` / `remove` / `attach(name, q)` / `detach(name, q)` | Scene objects. Boxes never collide with each other. |
| `has_collisions(q)` | Collision check of the whole robot plus scene objects. |

`grasp_candidates` returns TCP poses at the block centre with z (the approach) pointing down,
tilted by each of `GRASP_TILTS` away from the arm base, and the fingers (x) closing along the
block axis most across the line from the arm. Each tilt comes twice, with the fingers swapped.
`offset(tform, d)` backs a pose off by `d` along its approach axis.

`to_joint_trajectory` times each segment by its largest joint step over `v_max` (at least
0.1 s), positions only; `arm_controller` interpolates between them.

## Frames

- Planning is in `base_link`; the planning group `arm` is the chain `g_base → tcp`.
- `tcp` is a fixed link 45 mm along `gripper_base` z, between the fingertips (`emma_description`).
- The base, wheels and fingers stay at their default positions (fingers open) in collision checks.

## Collision model

roboplan builds its collision model from the URDF, with three planning-side changes made by
`load_urdf_for_planning`. The URDF itself is unchanged.

- **Mesh units**: coal ignores the COLLADA `<unit>` tag, so the gripper meshes (millimetres)
  load 1000× too big and the myAGV meshes (inches) 39× too big. The planner sets `scale` on
  those mesh tags (`0.001` and `0.0254`).
- **Base collision**: the upstream myAGV links have visuals only. Their meshes are copied into
  collision elements so the arm stays out of the base.
- **Mimic finger**: Pinocchio rejects the `<mimic>` tag here (it needs the mimicked joint
  first in the tree), so it is dropped. The right finger becomes an independent joint held
  at 0 (open), which is what the fingers are during every motion.

Disabled pairs: all parent-child links (`allowAdjacentLinkCollisions`), and
`gripper_left`/`gripper_right` against `joint6`/`joint6_flange`. No SRDF; upstream's
`firefighter.srdf` names the wrong robot and has no gripper. Sampling 300 random arm
configurations found no other always-colliding pairs, and `HOME_Q` is collision free without
further exceptions.

Attached objects hang off `joint6_flange`, not `tcp`: roboplan places added geometry relative
to the parent *joint* of the given frame, so a box parented to `tcp` lands 79 mm off.

## Assumptions and caveats

- **No time parameterisation**: TOPPRA (`roboplan.toppra`) needs a joint limits YAML, which
  emma doesn't have yet. Trajectories use the fixed-speed timing above.
- **No multi-goal RRT**: roboplan 0.7 has no `planToAny`; `plan_to_any` solves IK and plans
  to each pose in turn and returns the first success.
- **Straight-line moves are joint interpolations**: over the few centimetres they are used for
  (pregrasp ↔ grasp) the TCP stays close to the line, but nothing enforces it.
- **Table and block constants** are design choices for the sim scene, not measurements. The
  reachability test (`test_block_reachable_from_home`) is where to tune them.
