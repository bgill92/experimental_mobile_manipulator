# emma_manipulation

Arm motion planning for **emma** with [roboplan](https://github.com/open-planning/roboplan) 0.7,
used as an in-process C++ library: collision-aware IK, RRT-Connect with path shortcutting, short
straight-line moves, and top-down grasp poses for a small block. The library (`ArmPlanner`,
`graspCandidates`, `toJointTrajectory`) has no ROS node in it; `move_arm` is the thin ROS layer
that runs a plan on the simulated (or real) `arm_controller`.

## Contents

| Path | What it is |
|---|---|
| `include/emma_manipulation/constants.hpp` | Arm joint names (`kArmJoints`), `kHomeQ`, `kLookQ`, frames, table/block geometry, grasp tilts, gripper positions. |
| `include/emma_manipulation/planner.hpp`, `src/planner.cpp` | `ArmPlanner`, `rewriteUrdfForPlanning`, `meshPackagePaths`. |
| `include/emma_manipulation/grasp.hpp`, `src/grasp.cpp` | `graspCandidates`, `offset`. Pure Eigen. |
| `include/emma_manipulation/trajectory.hpp`, `src/trajectory.cpp` | `toJointTrajectory`. |
| `src/move_arm.cpp` | `move_arm` executable. |
| `test/test_planner.cpp` | Planner checks against the real URDF (expanded by xacro at build time), including the reachability gate for the block and the look pose. |
| `test/test_grasp.cpp` | Grasp pose geometry. |

Run the tests with `pixi run test`.

## move_arm

Start the sim (`pixi run sim`), then in another terminal:

```bash
pixi run bash -c "source install/setup.bash && ros2 run emma_manipulation move_arm --pose 0.25 0 0.20 0 3.1416 0"
pixi run bash -c "source install/setup.bash && ros2 run emma_manipulation move_arm --linear 0.25 0 0.18 0 3.1416 0"
pixi run bash -c "source install/setup.bash && ros2 run emma_manipulation move_arm --home"
```

| Option | Effect |
|---|---|
| `--home` | Plan (RRT) to `kHomeQ`, the folded start pose. |
| `--joints q1 … q6` | Plan (RRT) to arm joint positions in rad, in `kArmJoints` order. |
| `--pose x y z [roll pitch yaw]` | Plan (IK + RRT) to a TCP pose in `base_link`. Angles are fixed-axis x, y, z in rad. `0 3.1416 0` points the gripper straight down. |
| `--linear x y z [roll pitch yaw]` | Short straight move to a TCP pose (IK seeded at the current state, then a joint-space interpolation). |
| `--no-table` | Leave the table box out of the planning scene. |
| `--v-max` | Max joint speed for the trajectory timing, rad/s (default 0.8). |
| `-h`, `--help` | Print the usage text. |

It reads `/robot_description` (latched) and `/joint_states`, sends the trajectory to
`/arm_controller/follow_joint_trajectory`, waits for the result, and logs the reached TCP pose
and its error. It exits 0 on success and 1 on anything else (bad arguments, no URDF or joint
state, no plan, goal rejected or failed); the error says which. In the sim the error is
typically 2–5 mm: the position actuators sag a little under gravity.

## C++ API

```cpp
#include "emma_manipulation/grasp.hpp"
#include "emma_manipulation/planner.hpp"
#include "emma_manipulation/trajectory.hpp"

using namespace emma_manipulation;

ArmPlanner planner(urdf_xml);                                    // base_link -> tcp
const Eigen::VectorXd q = ArmPlanner::armPositions(joint_state).value();  // kArmJoints order
planner.addBox("table", size_xyz, tform_in_base);                // full side lengths
const std::vector<Eigen::Matrix4d> grasps = graspCandidates(block_xyz, block_yaw, arm_base_xy);
std::vector<Eigen::Matrix4d> pregrasps;
for (const Eigen::Matrix4d& g : grasps) {
  pregrasps.push_back(offset(g, kPreOffset));
}
const tl::expected<ArmPlanner::PlanToAny, std::string> found = planner.planToAny(q, pregrasps);
if (found.has_value()) {
  const trajectory_msgs::msg::JointTrajectory traj = toJointTrajectory(found->path);
  const tl::expected<std::vector<Eigen::VectorXd>, std::string> down =
      planner.planLinear(found->path.back(), grasps[found->index]);
  if (down.has_value()) {
    planner.attach("block", down->back());                       // held objects move with the wrist
  }
}
```

Link against `emma_manipulation::emma_manipulation` (`find_package(emma_manipulation)`); its
roboplan, tinyxml2, Eigen and message dependencies come along transitively.

| Call | Returns |
|---|---|
| `ArmPlanner(urdf_xml, ArmPlannerOptions{base_frame = "base_link", tip_frame = "tcp", max_planning_time = 2.0, seed = nullopt})` | Planner for the chain `g_base → tip_frame`. `seed` makes IK, RRT and shortcutting repeatable. Throws `std::runtime_error` if the URDF cannot be loaded, `ament_index_cpp::PackageNotFoundError` if a mesh package is missing. Not thread-safe. |
| `ArmPlanner::armPositions(joint_state)` | The `kArmJoints` positions of a `JointState`, in order, or an error listing the missing joints. |
| `fullQ(q)` | The scene's full position vector with the arm slots replaced by `q`. |
| `clamp(q)` | `q` clamped into the joint limits. |
| `fk(q, frame = tip)` | 4×4 pose of `frame` in `base_link`. |
| `ik(tform, q_seed, local = false)` | Collision-free IK solution or `nullopt`. `local` disables random restarts. |
| `planJoint(q_start, q_goal)` | Shortcut RRT-Connect path (6-vectors, including the start), or an error (goal in collision, RRT failure). |
| `planToAny(q_start, tforms)` | `{path, index}` for the first pose in `tforms` that IK and RRT reach, or `"no candidate reachable"`. |
| `planLinear(q_start, tform, steps = 10, max_joint_jump = 1.0)` | Joint-interpolated path to `tform`, or an error if IK fails, jumps to another branch (a joint moves more than `max_joint_jump` rad) or a step collides. |
| `addBox` / `hasBox` / `remove` / `attach(name, q)` / `detach(name, q)` | Scene objects. Boxes never collide with each other. |
| `hasCollisions(q)` | Collision check of the whole robot plus scene objects. |
| `scene()` | The `roboplan::Scene`, for thread-safe FK with a caller-owned `pinocchio::Data`. |

Recoverable failures come back as `tl::expected<T, std::string>` (the Python version returned
`None`); roboplan calls that only fail on programming errors (unknown names) throw.

`graspCandidates` returns TCP poses at the block centre with z (the approach) pointing down,
tilted by each of `kGraspTilts` away from the arm base, and the fingers (x) closing along the
block axis most across the line from the arm. Each tilt comes twice, with the fingers swapped.
`offset(tform, d)` backs a pose off by `d` along its approach axis.

`toJointTrajectory(path, v_max = 0.8, min_segment_time = 0.1)` drops the start waypoint and
times each segment by its largest joint step over `v_max` (at least `min_segment_time`),
positions only; `arm_controller` interpolates between them.

`planJoint` and `planLinear` clamp the start configuration into the joint limits. A measured
joint state can sit a hair past a limit (joint 6 reads 3.1416 against its 3.14159 limit after a
move to it), and roboplan's RRT rejects such a start outright.

## Frames

- Planning is in `base_link`; the planning group `arm` is the chain `g_base → tcp`.
- `tcp` is a fixed link 45 mm along `gripper_base` z, between the fingertips (`emma_description`).
- `kCameraFrame` is the wrist camera's `wrist_camera_color_optical_frame`.
- The base, wheels and fingers stay at their default positions (fingers open) in collision checks.

## Arm poses

| Constant | Value | What it is |
|---|---|---|
| `kHomeQ` | `[0, 2.259, -2.505, 0.3, 0, 0]` | Folded start pose, equal to the ros2_control `initial_value`s. Joint 4 is at 0.3 rather than -0.495 so the wrist camera clears the upper arm. |
| `kLookQ` | `[0.862, 0.771, -1.533, -0.695, -0.133, 0.855]` | Wrist camera 0.2 m from `kBlockStart`, optical axis 65° below horizontal. `kPlaceXyz` and a block a few cm off `kBlockStart` stay in view. |

`kLookQ` came from IK for the optical frame (through its fixed offset from the TCP) and is
hardcoded; the `LookPose` test re-solves it, prints the solution, and checks that the hardcoded
value is collision free, reachable from `kHomeQ` and has the block on the optical axis. At 45°
from 0.2 m the camera would sit almost over the arm base, and IK found no solution at 45° or 55°
from 0.17–0.23 m. 60–70° all work; 65° is in the middle. The camera is on the gripper's side, so
joint 1 turns 0.86 rad to bring it over the block.

## Collision model

roboplan builds its collision model from the URDF, with three planning-side changes made by
`rewriteUrdfForPlanning` (tinyxml2). The `robot_description` itself is unchanged.

- **Mesh units**: coal/assimp ignore the COLLADA `<unit>` tag, so the gripper meshes
  (millimetres) load 1000× too big and the myAGV meshes (inches) 39× too big. The planner sets
  `scale` on those mesh tags (`0.001` and `0.0254`).
- **Base collision**: the upstream myAGV links have visuals only. Their meshes are copied into
  collision elements so the arm stays out of the base.
- **Mimic finger**: Pinocchio rejects the `<mimic>` tag here (it needs the mimicked joint first
  in the tree), so it is dropped. The right finger becomes an independent joint held at 0
  (open), which is what the fingers are during every motion.

Disabled pairs: all parent-child links (`allowAdjacentLinkCollisions`), and
`gripper_left`/`gripper_right`/`wrist_camera_link` against `joint6`/`joint6_flange` (pairs naming
a link the URDF lacks, such as the camera with `camera:=none`, are skipped). No SRDF; upstream's
`firefighter.srdf` names the wrong robot and has no gripper. `kHomeQ` is collision free without
further exceptions.

Attached objects hang off `joint6_flange`, not `tcp`: roboplan places added geometry relative to
the parent *joint* of the given frame, so a box parented to `tcp` lands 79 mm off. An attached
box is excluded from collisions with `gripper_base`, `gripper_left` and `gripper_right`.

## Assumptions and caveats

- **No time parameterisation**: TOPPRA (`roboplan_toppra`) needs a joint limits YAML, which emma
  doesn't have yet. Trajectories use the fixed-speed timing above.
- **No multi-goal RRT**: roboplan 0.7 has none; `planToAny` solves IK and plans to each pose in
  turn and returns the first success.
- **Straight-line moves are joint interpolations**: over the few centimetres they are used for
  (pregrasp ↔ grasp) the TCP stays close to the line, but nothing enforces it.
- **Table and block constants** are design choices for the sim scene, not measurements. The
  reachability test (`BlockReachableFromHome`) is where to tune them.
- **Test URDF**: `test_planner` reads `emma.urdf`, expanded from `emma_description`'s xacro into
  the build directory at build time, so `emma_description`, `mycobot_description` and
  `myagv_description` must be built first (they are `exec_depend`s, which colcon honours).
