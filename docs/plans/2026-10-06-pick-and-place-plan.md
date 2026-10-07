# emma: sim pick-and-place with roboplan, py_trees and a wrist RGB-D camera

## Context

emma today is a URDF plus a MuJoCo/ros2_control sim (arm, gripper, mecanum base). Nothing plans, sequences or senses. This plan adds the first closed loop, built in this order per the user: **(1) manipulation** with roboplan (move the arm to joint configs and TCP poses), **(2) a py_trees behaviour tree** that sequences those moves into a scripted pick-and-place of a block at a known pose, **(3) perception** from a simulated wrist depth camera that replaces the known pose with a detected one. All in sim with the base parked.

Decisions made with the user: planner = **open-planning/roboplan 0.7.0** as an in-process Python library (no action server, no custom msgs); perception = classical HSV + depth backprojection (learned detector later); three new ament_python packages; bare table scene (MolmoSpaces room later); sim camera stands in for a real **Orbbec Gemini 305** (4 cm min range vs D405's 7 cm, 68 g, $229, USB-C, OrbbecSDK_ROS2 v2; same topic layout so the real driver drops in later).

Everything needed is on the pixi channels (verified with `pixi search`): `ros-lyrical-roboplan` 0.7.0 / `roboplan-python` 0.7.0, `ros-lyrical-py-trees(-ros)` 2.6.0, `ros-lyrical-mujoco-ros2-control-plugins` 0.1.2 (CameraPlugin, already installed transitively), `ros-lyrical-cv-bridge`, `ros-lyrical-tf2-geometry-msgs`. `roboplan-ros` is not packaged and has no planning server anyway, so we write a ~150-line wrapper.

## Verified facts that shape the design

- Gripper (from finger meshes): inner faces at x = ±7.5 mm open, ±0.5 mm closed → **15 mm max opening, 1 mm closed**; fingertips end at z = 48.5 mm from `gripper_base`; `gripper_base` mesh free side is +y. So the demo block is a **10 mm cube**. Real gripper opening is unverified (upstream travel value) — caveat.
- Arm/gripper geoms are `contype=0 conaffinity=0` → nothing can be grasped today. Generated MJCF geoms are unnamed, so finger collision bits must be set in `postprocess_mjcf.py`, not via `modify_element`.
- myAGV body ends at x ≈ 0.164, top z = 0.132. A crude FK sweep says a strictly vertical top grasp at table height 0.15 m is marginal; grasps tilted 20–35° toward the robot are comfortable → grasp tilt is a candidate list.
- roboplan Python API: `Scene(name, loadUrdfSceneDescriptionFromXml(urdf, package_paths))`, `addGroupFromChain`, `allowAdjacentLinkCollisions`, `setCollisions`, `addBoxGeometry(name, parent, Box, tform, rgba)`, `hasCollisions(q)`, `SimpleIk(scene, SimpleIkOptions(group_name, max_iters, check_collisions))`, `CartesianConfiguration{base_frame,tip_frame,tform}`, `RRT(scene, RRTOptions(group_name, collision_check_step_size, max_planning_time, rrt_connect)).plan/planToAny`, `PathShortcutter`, `attachObject/detachObject`, `PathParameterizerTOPPRA` (needs a limits YAML; deferred).
- py_trees_ros 2.6: `action_clients.FromBlackboard(name, action_type, action_name, key, ...)`, `FromConstant(name, action_type, action_name, action_goal, ...)`, `subscribers.ToBlackboard(name, topic_name, topic_type, qos_profile, blackboard_variables, clearing_policy)`, `trees.BehaviourTree(root).setup(node_name, timeout)` + `tick_tock(period_ms)`.
- Converter (`make_mjcf_from_robot_description.py`) adds a MuJoCo camera at any site named in `mujoco_inputs.xml` `<processed_inputs><camera site=... name=... fovy=... resolution=.../>`; sites are named after URDF links; it applies the ROS-optical → MuJoCo rotation itself.
- `CameraPlugin` auto-configures from MJCF cameras. YAML passed to `ros2_control_node` via `ParameterFile`: `mujoco_plugins.<plugin>.type: mujoco_ros2_control_plugins/CameraPlugin`, `camera_publish_rate`, per-camera `frame_name/info_topic/image_topic/depth_topic`. Depth = 32FC1. Headless uses EGL.

## Constants (one module: `emma_manipulation/emma_manipulation/constants.py`)

| Name | Value |
|---|---|
| `ARM_JOINTS` | the six names in `controllers.yaml` order |
| `HOME_Q` | `[0, 2.259, -2.505, -0.495, 0, 0]` (= ros2_control initial_values) |
| `LOOK_Q` | found once by IK in the M3 test, then hardcoded |
| `TABLE` | box centre `(0.30, 0, 0.075)`, half-size `(0.10, 0.15, 0.075)` → top z = 0.15, x 0.20..0.40 (clears AGV body) |
| `BLOCK_SIZE` | 0.010 m, 5 g, red |
| `BLOCK_START` | `(0.25, 0.0, 0.155)` in `base_link` |
| `PLACE_XYZ` | `(0.25, 0.08, 0.155)` |
| `TCP` | fixed link `tcp` at `gripper_base` + `(0, 0, 0.045)` (fingertip centre) |
| `PRE_OFFSET` | 0.05 m along −approach (pregrasp / retreat / preplace) |
| `GRASP_TILTS` | `[0°, 20°, 35°]` toward the robot, tried via `planToAny` |
| `GRIPPER_OPEN / CLOSED` | `0.0` / `-0.007` |

## Milestones (each independently verifiable, commit each to main, update READMEs each time)

### M1 — Manipulation: `tcp` link + `src/emma_manipulation` (roboplan wrapper)

**URDF** (`src/emma_description/urdf/parallel_gripper.urdf.xacro`): `<link name="tcp"/>` + fixed joint `gripper_base_to_tcp`, origin `xyz="0 0 0.045"`. README TF tree gains `tcp`. Verify: `pixi run display`; `tf2_echo gripper_base tcp` → `0 0 0.045`.

**Deps**: `pixi add ros-lyrical-roboplan`; confirm `pixi run python -c "import roboplan"` (fall back to `roboplan-python` from conda-forge if the ROS package lacks bindings).

**Package**: `ros2 pkg create --build-type ament_python --license MIT emma_manipulation --dependencies rclpy sensor_msgs trajectory_msgs control_msgs geometry_msgs`.

`emma_manipulation/planner.py`:
```python
class ArmPlanner:
    def __init__(self, urdf_xml: str, base_frame="base_link", tip_frame="tcp")
    def set_q(self, joint_state: JointState) -> np.ndarray      # pick ARM_JOINTS by name
    def fk(self, q, frame="tcp") -> np.ndarray                   # 4x4
    def ik(self, tform, q_seed) -> np.ndarray | None             # SimpleIk, check_collisions=True
    def plan_joint(self, q_start, q_goal) -> list[np.ndarray] | None          # RRT.plan + shortcut
    def plan_to_any(self, q_start, tforms) -> list[np.ndarray] | None         # ik each, RRT.planToAny + shortcut
    def plan_linear(self, q_start, tform, steps=10) -> list[np.ndarray] | None  # ik seeded at q_start, joint lerp, hasCollisions per step
    def add_box(self, name, size_xyz, tform_in_base, rgba=...); remove(name); attach(name, frame="tcp"); detach(name)
def to_joint_trajectory(path, v_max=0.8) -> trajectory_msgs.msg.JointTrajectory
    # time_from_start += max|Δq|/v_max per segment (min 0.1 s); positions only, JTC interpolates. TOPPRA deferred (needs a limits YAML).
def load_urdf_for_planning(urdf_xml) -> tuple[str, list[str]]
    # package_paths via get_package_share_directory for emma_description, mycobot_description, myagv_description
```
Scene setup: `addGroupFromChain("arm", "g_base", "tcp")`, `allowAdjacentLinkCollisions()`, then `setCollisions(...)` to disable base↔arm pairs the home pose trips (the coarse `myagv_up` hull will intersect the folded arm) and gripper↔wrist pairs (`gripper_left/right` vs `joint6`, `joint6_flange`). No SRDF (upstream `firefighter.srdf` has the wrong robot name and no gripper; three calls replace it).
Risk: coal/assimp ignore the COLLADA `<unit>` tag → gripper meshes 1000× too big, base 39×. If `test_home_collision_free` fails, `load_urdf_for_planning` regex-adds `scale="0.001 ..."` to `parallel_gripper/*.dae` and `0.0254` to `myagv_*.dae` mesh tags, planning-side only.

`emma_manipulation/grasp.py`: `grasp_candidates(block_xyz, block_yaw, arm_base_xy, tilts=GRASP_TILTS) -> list[4x4]` (finger axis = block axis most perpendicular to base→block; approach z = −e_z tilted by θ away from the base; y = z × x); `offset(tform, dist)` translates along −z. Pure numpy.

`emma_manipulation/move_arm.py` (console script `move_arm`, the manual check for this milestone): reads `/robot_description` (transient_local) and `/joint_states`, builds `ArmPlanner`, adds `TABLE` box, then one of `--joints q1..q6`, `--pose x y z [roll pitch yaw]` (RRT), `--linear x y z ...`, `--home`; sends the trajectory with an `rclpy.action.ActionClient(FollowJointTrajectory)` to `/arm_controller/follow_joint_trajectory`, waits for the result, prints final TCP pose error. Keeps ROS out of `planner.py`.

`test/test_planner.py` (pytest; xacro via `xacro.process_file(...).toxml()`): (1) `fk(HOME_Q)` finite, `hasCollisions(HOME_Q)` False; (2) IK round trip < 1 mm; (3) `plan_to_any(HOME_Q, grasp_candidates(BLOCK_START, 0, ...))` non-empty with table + block boxes in the scene — the reachability gate; tune `TABLE`/`GRASP_TILTS` here, not in sim; (4) `plan_linear` grasp↔pregrasp; (5) `to_joint_trajectory` monotonic times, `joint_names == ARM_JOINTS`.

`pixi.toml`: `test = colcon test --packages-select emma_manipulation && colcon test-result --verbose`.

Verify: `pixi run test`; `pixi run sim`, then `move_arm --pose 0.25 0 0.20 0 3.1416 0` reaches within ~5 mm, `move_arm --home` returns. README: API, frames, collision pairs, mesh-unit caveat, TOPPRA deferred.

### M2 — Behaviour tree: scripted pick-and-place at a known block pose (`src/emma_behaviors` + sim changes)

Perception is not in yet, so the tree uses `BLOCK_START` as the block pose. The sim needs a block it can actually grasp.

**Sim** (`src/emma_simulation`):
1. `scripts/postprocess_mjcf.py`: new step `set_finger_collisions(root)`: every `<geom class="collision">` whose mesh is `gripper_left`/`gripper_right` gets `contype="4" conaffinity="2" condim="4" friction="1.5 0.02 0.0005" solref="0.005 1" solimp="0.95 0.99 0.001"`. Nothing else changes, so the bare sim behaves as before.
2. New `mujoco/pick_scene.xml`: copy of `scene.xml` plus
   ```xml
   <option noslip_iterations="3"/>
   <geom name="table" type="box" pos="0.30 0 0.075" size="0.10 0.15 0.075" rgba="0.55 0.4 0.25 1" contype="1" conaffinity="6"/>
   <body name="block" pos="0.25 0 0.155"><freejoint name="block_joint"/>
     <geom name="block" type="box" size="0.005 0.005 0.005" mass="0.005" rgba="0.9 0.1 0.1 1"
           contype="2" conaffinity="5" condim="4" friction="1.5 0.02 0.0005" solref="0.005 1" solimp="0.95 0.99 0.001"/></body>
   ```
   Bit scheme: bit1 floor/structure, bit2 graspable objects, bit4 fingers. Floor `1/0`, rollers `0/1` (unchanged), table `1/6`, block `2/5`, fingers `4/2`. Fingers never hit each other or the arm. Block body after the robot include. kp 100 gives ~0.25 N per finger → ~0.75 N friction vs 0.05 N weight; raise kp to 300 only if it slips.
   `molmospaces_scene.py`: its geom-remap loop would overwrite finger contype; add `if geom.contype: continue`.
3. `pixi run gen-mjcf`; commit regenerated MJCF. README: contents (+`pick_scene.xml`), collision-bits table, grasp caveats.

**Deps**: `pixi add ros-lyrical-py-trees ros-lyrical-py-trees-ros ros-lyrical-py-trees-ros-interfaces`.

**Package**: `ros2 pkg create --build-type ament_python --license MIT emma_behaviors --dependencies rclpy py_trees py_trees_ros emma_manipulation control_msgs sensor_msgs geometry_msgs`.

`emma_behaviors/behaviours.py` (custom behaviours take `planner` in ctor, `self.node` from `setup(**kwargs)`):
- `PlanToJoints(name, planner, q_goal)` → `bb.trajectory = FollowJointTrajectory.Goal(trajectory=to_joint_trajectory(path))`, FAILURE if None.
- `PlanToTcp(name, planner, key, linear: bool)` → `bb[key]` is one 4×4 or a list → `plan_linear` / `plan_to_any`.
- `ComputeGraspPoses(name, planner)` → from `bb.block_pose` (PoseStamped) writes `bb.grasp_candidates`, `bb.pregrasp_candidates`, `bb.place`, `bb.preplace`; adds the block box to the planner scene; records the chosen candidate index after the first successful plan so grasp/lift reuse that tilt.
- `AttachBlock` / `DetachBlock` → `planner.attach/detach("block")`.
- `CheckPlaced(name, tol=0.02)` → compares `bb.block_pose` to `PLACE_XYZ`. In M2 this reads the sim ground truth; add `FreeJointStatePublisherPlugin` to a `config/mujoco_plugins.yaml` (passed via `ParameterFile` in `sim.launch.py`) and a tiny `ToBlackboard` on its topic, or skip `CheckPlaced` until M3 — pick the plugin route only if it is a 10-line add; otherwise defer.
- Stock: `Execute = FromBlackboard("execute", FollowJointTrajectory, "/arm_controller/follow_joint_trajectory", key="trajectory")`; `OpenGripper/CloseGripper = FromConstant(..., ParallelGripperCommand, "/gripper_action_controller/gripper_cmd", goal name=["gripper_controller"], position=[0.0]/[-0.007])`; `ToBlackboard("joints2bb", "/joint_states", JointState, ..., {"joint_state": None}, clearing_policy=NEVER)`.
- `SetBlockPose(name, xyz, yaw)` → writes a constant `bb.block_pose` (M2 stand-in for perception; removed in M3).

Tree (`emma_behaviors/pick_and_place.py`):
```
Parallel(SuccessOnSelected([task]), synchronise=False)
├─ joints2bb
└─ task: Selector(memory=True)
   ├─ Retry(2): Sequence(memory=True)
   │   SetBlockPose(BLOCK_START)            # M3: PlanToJoints(LOOK_Q) → Execute → Timer → Timeout(block2bb)
   │   → ComputeGraspPoses
   │   → OpenGripper → PlanToTcp(pregrasp_candidates, rrt) → Execute
   │   → PlanToTcp(grasp, linear) → Execute → CloseGripper → AttachBlock
   │   → PlanToTcp(pregrasp, linear) → Execute → PlanToTcp(preplace, rrt) → Execute
   │   → PlanToTcp(place, linear) → Execute → OpenGripper → DetachBlock
   │   → PlanToTcp(preplace, linear) → Execute → [CheckPlaced]
   │   → PlanToJoints(HOME_Q) → Execute
   └─ recover: OpenGripper → DetachBlock → PlanToJoints(HOME_Q) → Execute → Failure
```
`main()`: one-shot `/robot_description` read (`std_msgs/String`, transient_local); `planner = ArmPlanner(urdf)`; `add_box("table")`; `tree.setup(node_name="pick_and_place", timeout=15)`; `tick_tock(period_ms=200, post_tick_handler=stop_when_done)`; exit 0 on root SUCCESS, 1 on FAILURE. Planning is synchronous inside a tick (RRT budget ≤ 2 s), fine at 5 Hz.

`launch/pick_and_place.launch.py`: include `sim.launch.py` with `mujoco_model:=pick_scene.xml` + `pick_and_place` node (`on_exit=Shutdown()`); args `headless`, `rviz` passthrough. (M3 adds the detector node.)

`pixi.toml` tasks: `pick-demo` (launch, depends-on build), `pick-check = timeout 240 ros2 launch ... headless:=true rviz:=false` (exit 0 only if the tree reaches SUCCESS).

Verify: `pixi run pick-demo` → arm opens, descends, closes, block lifts with the fingers, lands near `PLACE_XYZ`, arm homes, exit 0. `pixi run sim` (bare) and `check_mecanum.py` unchanged. README: tree diagram, blackboard keys, launch args, failure handling, `py-trees-tree-viewer` hint.

### M3 — Perception: wrist camera in URDF + MuJoCo, `src/emma_perception`, tree swap

**URDF** (`parallel_gripper.urdf.xacro`, inside `<xacro:if value="${'$(arg camera)' == 'gemini305'}">`; declare `<xacro:arg name="camera" default="gemini305"/>` in `emma.urdf.xacro` next to `gripper`):
- `wrist_camera_link`: visual+collision `<box size="0.023 0.042 0.042"/>` (REP 103, x forward), mass 0.068, box inertia. Fixed joint from `gripper_base`, origin `xyz="0 0.037 0" rpy="0 -1.309 -1.5708"` (camera x = gripper +z tilted 15° toward the fingers; body just outside the +y edge of the gripper_base mesh).
- `wrist_camera_color_optical_frame`: fixed child, `rpy="-1.5708 0 -1.5708"` (z forward, x right, y down). Fingertips ~29° below the optical axis, inside the 32.5° half-VFOV → bottom edge of the image, ~5 cm from the lens (> 4 cm min range).
- Planner: add `wrist_camera_link` to the gripper↔wrist disabled-collision pairs.
README: TF tree, `camera` arg, assumptions (mount/tilt are design choices; one camera stands in for aligned RGB-D; fovy 65° at 640×400 ≈ 91° HFOV between the 94° RGB / 88° depth specs).

**Sim**:
1. `mujoco_inputs.xml` `<processed_inputs>`: `<camera site="wrist_camera_color_optical_frame" name="wrist_camera" fovy="65" mode="fixed" resolution="640 400"/>`. `gen-mjcf` now requires the camera site (converter raises otherwise) — note in README.
2. `config/camera.yaml` (CameraPlugin schema; name `wrist_camera`, frame `wrist_camera_color_optical_frame`, topics `/wrist_camera/color/image_raw`, `/wrist_camera/color/camera_info`, `/wrist_camera/depth/image_raw`, 5 Hz). `sim.launch.py`: append `ParameterFile(camera_file)` to `ros2_control_node` params (harmless without cameras).
3. `pixi add ros-lyrical-mujoco-ros2-control-plugins`; `package.xml` exec_depend. `pixi run gen-mjcf`, commit.
Verify: `ros2 topic hz /wrist_camera/depth/image_raw` ≈ 5; encoding `32FC1`, metres (echo a pixel at known distance); image shows fingertips at the bottom edge. README: contents (+`camera.yaml`), camera topics table, EGL headless caveat.

**Package**: `ros2 pkg create --build-type ament_python --license MIT emma_perception --dependencies rclpy sensor_msgs geometry_msgs cv_bridge tf2_ros tf2_geometry_msgs message_filters`; `pixi add ros-lyrical-cv-bridge ros-lyrical-tf2-geometry-msgs`.

`emma_perception/block_detector.py`:
- Pure functions: `segment(bgr, lower, upper) -> mask` (HSV `inRange`; red wraps, so if `lower[0]==0` also take `[170..180]`); `locate(mask, depth, K, min_area=30) -> (u, v, d, yaw_px) | None` (largest contour, `cv2.minAreaRect`, `d = nanmedian(depth[mask>0])`, reject outside 0.04–0.6 m); `backproject(u, v, d, K) -> [(u-cx)d/fx, (v-cy)d/fy, d]`.
- `BlockDetector(Node)`: params `hsv_lower=[0,120,70]`, `hsv_upper=[10,255,255]`, `min_area_px`, `target_frame="base_link"`, `cube_size=0.01`, `debug=False`. K from `camera_info`; `ApproximateTimeSynchronizer([color, depth], 5, 0.05)`; backproject centre + a second point along the minAreaRect axis; `PoseStamped` in optical frame at image stamp → `tf2_geometry_msgs.do_transform_pose_stamped` (0.2 s timeout, sim time) → `z -= cube_size/2` (we see the top face) → yaw from the transformed axis, roll/pitch 0 → publish `/block_pose`. Debug image on `/block_detector/debug_image` when `debug`. No debouncing; the tree samples once after settling.

`test/test_block_detector.py`: synthetic 640×400 image, red 20×20 square at (360, 170), flat depth 0.25, `K=[400,400,320,200]` → centre within 0.5 px, `backproject ≈ (0.025, -0.01875, 0.25)`; rotated square → yaw ≈ 30° mod 90°. Add to `pixi run test`.

**Tree swap** (`emma_behaviors`): replace `SetBlockPose` with `PlanToJoints(LOOK_Q) → Execute → Timer(1.0) → Timeout(10)(block2bb)` where `block2bb = ToBlackboard("block2bb", "/block_pose", PoseStamped, ..., {"block_pose": None}, clearing_policy=ON_INITIALISE)` (RUNNING until a fresh message = one-shot sample). Re-run the same sample before `CheckPlaced` (now through perception, so the ground-truth plugin from M2 can go). `LOOK_Q`: add planner test (6): IK with tip `wrist_camera_color_optical_frame`, optical axis ~45° down at `BLOCK_START` from ~0.2 m → print → hardcode. Launch adds the `block_detector` node.

Verify: `ros2 topic echo /block_pose` with the arm at `LOOK_Q` → within ~1 cm of `BLOCK_START`; `pixi run pick-demo` succeeds from perception; `pixi run pick-check` exits 0 headless. Move the block a few cm in `pick_scene.xml` and rerun: still succeeds.

### M4 — Docs sweep

Root `README.md`: packages list (+3), "Pick and place in sim" section (`pick-demo`, `pick-check`, `test`). Confirm each package README matches its final state (user preference: audit READMEs after every change).

## Verification (end to end, after M3)

1. `pixi run test` → planner + detector unit tests green.
2. `pixi run pick-demo` → look pose, `/block_pose` appears, pick, place, home; exit 0.
3. `pixi run pick-check` → headless exit 0 within 240 s.
4. `pixi run sim` (bare scene) still works; `check_mecanum.py` still passes.

## Risks / open questions

1. Reachability of the table/block constants: gated by M1 test (3); knobs are `TABLE`, `BLOCK_START`, `GRASP_TILTS`.
2. Gripper opening 15 mm comes from upstream's unverified 7 mm travel; the real gripper likely opens wider. Measure before hardware; cube stays 10 mm in sim.
3. COLLADA units in coal/pinocchio (fallback described in M1).
4. `ros-lyrical-roboplan` Python bindings vs `roboplan-python` (1-minute check at the start of M1).
5. Grasp stability with kp 100 (fallback kp 300 in `mujoco_inputs.xml`).
6. `ToBlackboard` ON_INITIALISE re-sampling and `Retry` semantics in py_trees 2.6 — confirm on first run.
7. Later: YOLO/YOLO-World node behind the same `/block_pose` topic; real Gemini 305 via OrbbecSDK_ROS2 publishing the same topic names; MolmoSpaces room scene; TOPPRA once a joint-limits YAML exists; base motion in the tree.
