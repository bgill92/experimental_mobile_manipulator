# emma_simulation

MuJoCo simulation of emma through
[mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control). MuJoCo runs inside
the controller manager as a ros2_control hardware plugin, so the controllers are the same ones a
real robot would use.

**Scope:** the arm, gripper, and mecanum base are controlled. The base is a free body
standing on four mecanum wheels modelled with passive rollers. The wrist camera publishes colour
and depth images.

## Contents

| Path | What it is |
|---|---|
| `launch/sim.launch.py` | Starts MuJoCo + controller manager, `robot_state_publisher`, controller spawner, and RViz. The arm starts folded back over the base (`initial_value`s in `emma.urdf.xacro`). |
| `config/controllers.yaml` | Controller manager and controller parameters. |
| `config/wheel_pids.yaml` | Velocity PID gains the MuJoCo plugin uses to drive the wheel motors. |
| `config/mujoco_plugins.yaml` | mujoco_ros2_control plugins: `FreeJointStatePublisherPlugin` publishes ground-truth poses of free bodies on `/free_joint_states`. |
| `config/camera.yaml` | `CameraPlugin` settings for the wrist camera: frame, topics, 5 Hz. |
| `mujoco/scene.xml` | Top-level MJCF: floor, lights, visual settings; includes the robot model. |
| `mujoco/pick_scene.xml` | `scene.xml` plus a table and a graspable 10 mm block, for the pick-and-place demo in `emma_behaviors`. |
| `mujoco/mujoco_inputs.xml` | Converter input: actuators, mimic-finger equality, joint damping and armature, geom defaults, the wrist camera. |
| `mujoco/mujoco_description_formatted.xml` | **Generated** robot MJCF. Do not hand-edit; regenerate. |
| `mujoco/assets/` | **Generated** OBJ meshes and textures (~33 MB). |
| `rviz/sim.rviz` | RViz config rooted at `odom` with an Odometry display, so the base is seen moving. |
| `scripts/gen_mjcf.sh` | Regenerates the two generated items above from the URDF. |
| `scripts/postprocess_mjcf.py` | Fixes the converter output and adds the mecanum rollers (see below). |
| `scripts/check_mecanum.py` | Physics-only check that the base drives forward, strafes, and turns the commanded way. |
| `scripts/molmospaces_scene.py` | Downloads a MolmoSpaces room and writes a scene with emma in it (see *Living room scene*). |
| `mujoco/molmospaces/` | **Downloaded and generated**, gitignored: rooms, their object assets, and the composed `<plan>_emma.xml` scenes. |

## Running

```bash
pixi run sim                               # MuJoCo viewer + RViz
pixi run sim headless:=true rviz:=false    # no windows
```

| Launch argument | Default | Effect |
|---|---|---|
| `headless` | `false` | Run MuJoCo without its viewer. |
| `rviz` | `true` | Start RViz with `rviz/sim.rviz` (robot model + odometry trail). |
| `mujoco_model` | `share/emma_simulation/mujoco/scene.xml` | MJCF scene to load. `pick_scene.xml` adds the table and block. |

`pixi run sim` builds the workspace first, so freshly generated MJCF files get installed.

### Controllers

| Controller | Type | Interface |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publishes `/joint_states` (all 12 joints, including the mimic finger and wheels). |
| `arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | Action `/arm_controller/follow_joint_trajectory`; position commands on the six arm joints; partial goals allowed. |
| `gripper_action_controller` | `parallel_gripper_action_controller/GripperActionController` | Action `/gripper_action_controller/gripper_cmd` (`control_msgs/action/ParallelGripperCommand`) on `gripper_controller`. |
| `mecanum_drive_controller` | `mecanum_drive_controller/MecanumDriveController` | Topic `/mecanum_drive_controller/reference` (`geometry_msgs/msg/TwistStamped`); velocity commands on the four wheels. Publishes `/mecanum_drive_controller/odometry` and the `odom → base_footprint` TF. Stops 0.5 s after the last reference. |

The gripper controller isn't called `gripper_controller` because that is the gripper joint's name.
Its `goal_tolerance` is 1 mm (the 10 mm default is wider than the 7 mm stroke, so every goal would
succeed at once), and `allow_stalling` is on, so fingers stopped by a grasped object report
success after 0.5 s instead of aborting.

### Example goals

Run these in `pixi shell` after `source install/setup.bash`. Paste each as a single line.

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6], points: [{positions: [0.5, -0.5, 0.5, 0.0, 0.3, 0.0], time_from_start: {sec: 3}}]}}"

ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: [gripper_controller], position: [-0.007]}}"   # 0.0 opens

ros2 topic pub -r 10 /mecanum_drive_controller/reference geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1, y: 0.0}, angular: {z: 0.0}}}"   # m/s and rad/s
```

### Wrist camera

`CameraPlugin` (from `mujoco_ros2_control_plugins`, configured in `config/camera.yaml`) renders
the MJCF camera `wrist_camera` and publishes:

| Topic | Type | Notes |
|---|---|---|
| `/wrist_camera/color/image_raw` | `sensor_msgs/Image` | `rgb8`, 640 × 400 |
| `/wrist_camera/depth/image_raw` | `sensor_msgs/Image` | `32FC1`, metres along the optical axis |
| `/wrist_camera/color/camera_info` | `sensor_msgs/CameraInfo` | fx = fy ≈ 313.9, cx 320, cy 200, no distortion |

All three are stamped in sim time, in `wrist_camera_color_optical_frame`, at 5 Hz. One camera
stands in for the real sensor's aligned colour and depth, so the depth image is already
registered to the colour one. The topic names follow the Orbbec ROS 2 driver's layout so a real
Gemini 305 can replace the sim camera later. Rendering uses GLFW when a display is available and
falls back to EGL (headless OpenGL) when it is not, so `headless:=true` without a display still
gets images. With the arm folded at the start pose the camera sees the sky and the fingertips;
`emma_manipulation`'s `LOOK_Q` points it at the table.

Other useful topics:
- `/simulator/floating_base_state`: ground-truth base pose (`nav_msgs/Odometry`, frame `odom`).
  Compare it with `/mecanum_drive_controller/odometry` to see wheel-odometry drift.
- `/free_joint_states`: ground-truth pose of every free body
  (`mujoco_ros2_control_msgs/msg/FreeJointStateArray`), relative to the robot's `base_footprint`
  body, at 10 Hz. In `pick_scene.xml` that includes the `block`; the robot's own free joint always
  reads identity.
- `/clock`: sim time. Every node runs with `use_sim_time`, so pausing MuJoCo pauses the controllers.

### Living room scene

emma can also run in a furnished room from [MolmoSpaces](https://github.com/allenai/molmospaces)
(AI2's iTHOR rooms converted to MJCF; data CC BY 4.0, not redistributed here):

```bash
pixi run molmospaces-scene   # first run downloads ~30 MB on disk (~550 MB transferred)
pixi run sim mujoco_model:=$PWD/src/emma_simulation/mujoco/molmospaces/scenes/ithor/FloorPlan201_emma.xml
```

The default is living room `FloorPlan201`, with emma in an open strip beside the sofa facing +y.
Other rooms take the plan name and a spawn pose in the room's world frame:
`pixi run molmospaces-scene FloorPlan205 x y yaw`. `--freeze-kg` sets the static threshold (see
below). iTHOR living rooms are `FloorPlan201`-`230`, bedrooms `301`-`330`, kitchens `1`-`30`,
bathrooms `401`-`430`. Pick the pose by rendering or viewing the bare room
(`<plan>_physics.xml`); a pose inside furniture leaves emma stuck on top of it.

The script fetches only the room archive and the object files it references, using HTTP range
requests into the dataset's shard tars. It then attaches emma's MJCF to the room with `MjSpec` and
writes `<plan>_emma.xml`, with asset paths made absolute so both models' mesh directories survive.
It also remaps emma's collision bits to fit the room's:

| Geoms | Room scene | Why |
|---|---|---|
| Rollers | `conaffinity` 1 → 9 | The room's floor and walls have `contype` 8. |
| Arm collision meshes | `contype` 0 → 2 | The arm hits walls, furniture and loose objects. |
| Chassis collision meshes | `contype` 0 → 4; the floor drops bit 4 | The chassis hits furniture, but its hull reaches the ground and would drag on the floor. |
| Finger collision meshes | unchanged (`contype` 4, `conaffinity` 2) | They already collide; the room's objects accept every bit, so the fingers touch them too. |

To keep the room fast, loose bodies of at least `--freeze-kg` (default 4 kg) lose their free joint
and become part of the static world. In `FloorPlan201` that is the 16 pieces of furniture: sofa,
armchairs, tables, TV stand, chairs and floor lamp. MuJoCo skips contacts between static bodies,
and emma can't move furniture this heavy anyway. Drawers and doors keep their own joints. The 16
small objects (books, laptop, plant, remote...) stay loose, and MuJoCo sleep is enabled so they
cost nothing while resting and wake when touched. `--freeze-kg inf` keeps everything loose.

emma's own geoms still ignore each other, so the arm still passes through the base. Rerun the
script after `pixi run gen-mjcf`, since the composed scene copies the robot model.
`check_mecanum.py <plan>_emma.xml` passes in `FloorPlan201`.

### Physics only (no ROS)

```bash
pixi run python -m mujoco.viewer --mjcf=src/emma_simulation/mujoco/scene.xml
pixi run python src/emma_simulation/scripts/check_mecanum.py   # drive check; exits 1 on failure
```

In the viewer, enable geom group 3 to see the roller spheres.

## Regenerating the MuJoCo model

```bash
pixi run gen-mjcf
```

Rerun after changing anything in `emma_description/urdf/` or `mujoco/mujoco_inputs.xml`.
The converter adds the wrist camera at the `wrist_camera_color_optical_frame` site and fails if
the URDF has no such link, so generate with the default `camera:=gemini305`.
`scene.xml` is included at load time, so editing it needs no regeneration. The steps:

1. **xacro**: expands `emma.urdf.xacro` with defaults (`ros2_control:=none`).
2. **Converter**: mujoco_ros2_control's `make_mjcf_from_robot_description.py` runs with
   `--add_free_joint` in a temp directory. It converts the DAE meshes to OBJ, merges
   `mujoco_inputs.xml`, and turns the URDF root into a free body (`floating_base_joint`).
3. **`postprocess_mjcf.py`**:
   - restores the base `<inertial>` the converter drops, copied from the URDF it compiled;
   - rescales the gripper (mm) and myAGV (inch) meshes, because the converter ignores COLLADA
     units and RViz does not;
   - adds 12 roller spheres to each wheel body, offset by half a roller pitch (see *Mecanum
     wheels* below);
   - gives the finger collision meshes contact bits and a grippy contact (see *Contacts*).
4. **Copy back**: only `mujoco_description_formatted.xml` and the asset files it references go
   into `mujoco/`. The rest are converter intermediates and are discarded.

The URDF stays the only description. Everything the MJCF adds lives in `mujoco_inputs.xml`,
`scene.xml` or `postprocess_mjcf.py`, so regenerating never loses hand edits.

## Model details

- **Actuators**: named after their joints, because mujoco_ros2_control matches them by name.
  The arm and gripper use `position` actuators, and the wheels use `motor` actuators.
  - Arm: kp 50 on the first three joints, 20 on the wrist joints, `dampratio` 1, plus joint
    damping (1 / 0.5) and armature 0.01.
  - Gripper: kp 100.
  - Wheels: `motor` (torque) actuators, clamped to ±1 N·m. mujoco_ros2_control turns the
    velocity command into torque with the P gain in `config/wheel_pids.yaml`, once per
    controller cycle (100 Hz). A `velocity` actuator fights the roller contacts. Wheel joints
    get armature 0.002 to stand in for the gear motor's rotor inertia; without it the PID
    oscillates.
- **Mimic finger**: the URDF `<mimic>` is dropped by the converter. An `<equality><joint>` with
  `polycoef="0 -1 0 0 0"` replaces it. ros2_control treats the joint as passive and only reads its
  state.
- **Contacts**: every robot geom except the rollers and the fingers has `contype=0
  conaffinity=0`. The rollers have `conaffinity=1`, so they touch the floor. The arm does not
  collide with anything, itself included. The finger collision meshes (`gripper_left`,
  `gripper_right`) get `contype=4 conaffinity=2` from `postprocess_mjcf.py`, with `condim=4`,
  friction `1.5 0.02 0.0005` and a stiff `solref`/`solimp`, so they can pinch a block. Bits:

  | Bit | Meaning | Geoms (`contype` / `conaffinity`) |
  |---|---|---|
  | 1 | floor and structure | floor `1/0`, rollers `0/1`, `pick_scene.xml` table `1/6` |
  | 2 | graspable objects | `pick_scene.xml` block `2/5` |
  | 4 | fingers | fingers `4/2` |

  Two geoms touch when either one's `contype` shares a bit with the other's `conaffinity`. So the
  fingers touch the block and the table but not each other, the arm or the floor; the block rests
  on the table and the floor. In `scene.xml` there is nothing for the fingers to touch, so the
  bare sim behaves as before.
- **Mecanum wheels**: ported from
  [JunHeonYoon/mujoco_mecanum](https://github.com/JunHeonYoon/mujoco_mecanum) (MIT). Each wheel
  carries 12 passive spheres on hinges, so contact friction behaves like rollers. The port
  changes two things:
  - the spheres use the roller radius (upstream's generator uses the wheel radius);
  - the roller axes are set to 45°. Upstream's construction comes out at ~14° on a wheel this
    small, and the base then strafes at a quarter of the commanded speed.

  Opposite corners share a tilt, forming the X layout that `mecanum_drive_controller` assumes.
  `check_mecanum.py` catches a flipped tilt. The rollers sit half a pitch (15°) off the wheel's
  zero angle, so the robot starts resting on two rollers per wheel. With one roller straight
  down, the base starts balanced on it and rolls about 1 cm backwards in the first seconds.
- **Wrist camera**: `mujoco_inputs.xml` adds a `<camera>` (fovy 65°, 640 × 400) at the
  `wrist_camera_color_optical_frame` site; the converter turns the REP 103 optical frame into
  MuJoCo's camera convention. The camera body is a box with `contype=0`, like the rest of the
  arm.
- **Free base**: the plugin publishes the free joint's pose on
  `/simulator/floating_base_state`. `mecanum_drive_controller` publishes the `odom →
  base_footprint` TF from wheel odometry.
- **Shadows**: `scene.xml` sets `<map shadowclip="6"/>`. MuJoCo's default of 1 speckles the floor
  and the base with shadow acne. The key light uses `mode="trackcom"`, so its shadow map follows
  the robot once the base drives.

## Assumptions and caveats

- **Grasping is tuned for the 10 mm demo block**: the fingers open to 15 mm and close to 1 mm
  (inner faces, from the meshes). MuJoCo collides the convex hull of each finger mesh. The block
  is 5 g. Closed on it, the gripper actuator stops 4.5 mm short of its -7 mm target, so at kp 100
  it squeezes with about 0.45 N, which holds 0.05 N of weight with margin at friction 1.5
  (`noslip_iterations="3"` in `pick_scene.xml` also stops it creeping). Raise the gripper kp in
  `mujoco_inputs.xml` if a heavier object slips. The real gripper's opening is unverified.

- **Dynamics are approximate**: masses and inertias are estimates (see `emma_description`'s
  README), and the actuator gains were picked to hold pose and track goals, not to match the real
  servos.
  - With kp 50, `joint3_to_joint2` settles about 0.005 rad below its target under gravity.
  - Raise kp in `mujoco_inputs.xml` and regenerate if that matters.
- **Wheel geometry, motor torque and PID gains are estimates**: the wheel geometry comes from
  the base mesh (see `emma_description`'s README). In plain MuJoCo, `check_mecanum.py` reaches
  about 95% of the commanded motion on all three axes. The 1 N·m clamp and P 0.2 were not
  matched to the real motors.
- **Other sims on the same ROS domain break this one**: two sims publishing `/clock` make time
  jump backwards, and the wheel PID then throws `Pid is called with negative dt`. Set a
  distinct `ROS_DOMAIN_ID` if another sim is running.
- **Only the default robot variant is generated**: `model:=both gripper:=parallel`. The
  `<ros2_control>` block only exists for `model:=both`.
- **Generated files**: `mujoco/assets/` and `mujoco_description_formatted.xml` are
  generated but kept in the source tree, so the sim runs straight after a clone. Most of the size
  is the 13 MB myAGV base mesh.
- **Converter version**: built against `ros-lyrical-mujoco-ros2-control` 0.1.2. The converter's
  Python dependencies (`mujoco-python`, `trimesh`, `pycollada`, `obj2mjcf`) come from `pixi.toml`.
- **Real-time warning**: `Could not enable FIFO RT scheduling policy` at startup is harmless in
  simulation.
- **Room scenes are slower than the bare scene**: `FloorPlan201` with emma steps at about 15× real
  time in plain MuJoCo, against about 50× for `scene.xml`. With nothing frozen and sleep off it
  drops to about 2×: the 32 loose bodies resting on each other make ~240 contacts a step, and the
  room's 4 `noslip_iterations` take about 40% of the step. The room's solver options (elliptic
  cone, `noslip_iterations` 4, `impratio` 10) replace the defaults in `scene.xml`.
- **Frozen furniture doesn't move**: emma can't push a chair out of the way. Lower `--freeze-kg`
  for more speed, raise it (or `inf`) for more loose objects.
- **Room scenes use the room's floor**: the ceiling, walls and floor come from the room, and the
  floor sits at z = 0. The composed XML holds absolute paths, so regenerate it after moving the
  checkout.
- **Camera rendering costs CPU/GPU**: the plugin renders 5 colour and depth frames a second
  even when nothing reads them. Lower `camera_publish_rate` in `config/camera.yaml` if the sim
  runs slow.
- **Passive-joint warning**: `Unable to find the actuator 'gripper_base_to_gripper_right'` is
  expected, because that joint is the passive mimic finger.
