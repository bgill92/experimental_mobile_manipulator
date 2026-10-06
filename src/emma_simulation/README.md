# emma_simulation

MuJoCo simulation of emma through
[mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control). MuJoCo runs inside
the controller manager as a ros2_control hardware plugin, so the controllers are the same ones a
real robot would use.

**Scope:** the arm, gripper, and mecanum base are controlled. The base is a free body
standing on four mecanum wheels modelled with passive rollers.

## Contents

| Path | What it is |
|---|---|
| `launch/sim.launch.py` | Starts MuJoCo + controller manager, `robot_state_publisher`, controller spawner, and RViz. |
| `config/controllers.yaml` | Controller manager and controller parameters. |
| `config/wheel_pids.yaml` | Velocity PID gains the MuJoCo plugin uses to drive the wheel motors. |
| `mujoco/scene.xml` | Top-level MJCF: floor, lights, visual settings; includes the robot model. |
| `mujoco/mujoco_inputs.xml` | Converter input: actuators, mimic-finger equality, joint damping and armature, geom defaults. |
| `mujoco/mujoco_description_formatted.xml` | **Generated** robot MJCF. Do not hand-edit; regenerate. |
| `mujoco/assets/` | **Generated** OBJ meshes and textures (~33 MB). |
| `scripts/gen_mjcf.sh` | Regenerates the two generated items above from the URDF. |
| `scripts/postprocess_mjcf.py` | Fixes the converter output and adds the mecanum rollers (see below). |
| `scripts/check_mecanum.py` | Physics-only check that the base drives forward, strafes, and turns the commanded way. |

## Running

```bash
pixi run sim                               # MuJoCo viewer + RViz
pixi run sim headless:=true rviz:=false    # no windows
```

| Launch argument | Default | Effect |
|---|---|---|
| `headless` | `false` | Run MuJoCo without its viewer. |
| `rviz` | `true` | Start RViz with `emma_description`'s config. |
| `mujoco_model` | `share/emma_simulation/mujoco/scene.xml` | MJCF scene to load. |

`pixi run sim` builds the workspace first, so freshly generated MJCF files get installed.

### Controllers

| Controller | Type | Interface |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publishes `/joint_states` (all 12 joints, including the mimic finger and wheels). |
| `arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | Action `/arm_controller/follow_joint_trajectory`; position commands on the six arm joints; partial goals allowed. |
| `gripper_action_controller` | `parallel_gripper_action_controller/GripperActionController` | Action `/gripper_action_controller/gripper_cmd` (`control_msgs/action/ParallelGripperCommand`) on `gripper_controller`. |
| `mecanum_drive_controller` | `mecanum_drive_controller/MecanumDriveController` | Topic `/mecanum_drive_controller/reference` (`geometry_msgs/msg/TwistStamped`); velocity commands on the four wheels. Publishes `/mecanum_drive_controller/odometry` and the `odom → base_footprint` TF. Stops 0.5 s after the last reference. |

The gripper controller isn't called `gripper_controller` because that is the gripper joint's name.

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

Other useful topics:
- `/simulator/floating_base_state`: ground-truth base pose (`nav_msgs/Odometry`, frame `odom`).
  Compare it with `/mecanum_drive_controller/odometry` to see wheel-odometry drift.
- `/clock`: sim time. Every node runs with `use_sim_time`, so pausing MuJoCo pauses the controllers.

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
`scene.xml` is included at load time, so editing it needs no regeneration. The steps:

1. **xacro**: expands `emma.urdf.xacro` with defaults (`ros2_control:=none`).
2. **Converter**: mujoco_ros2_control's `make_mjcf_from_robot_description.py` runs with
   `--add_free_joint` in a temp directory. It converts the DAE meshes to OBJ, merges
   `mujoco_inputs.xml`, and turns the URDF root into a free body (`floating_base_joint`).
3. **`postprocess_mjcf.py`**:
   - restores the base `<inertial>` the converter drops, copied from the URDF it compiled;
   - rescales the gripper (mm) and myAGV (inch) meshes, because the converter ignores COLLADA
     units and RViz does not;
   - adds 12 roller spheres to each wheel body (see *Mecanum wheels* below).
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
- **Contacts**: every robot geom except the rollers has `contype=0 conaffinity=0`. The rollers
  have `conaffinity=1`, so the only contacts are the rollers against the floor. The arm does not collide with anything, itself included.
- **Mecanum wheels**: ported from
  [JunHeonYoon/mujoco_mecanum](https://github.com/JunHeonYoon/mujoco_mecanum) (MIT). Each wheel
  carries 12 passive spheres on hinges, so contact friction behaves like rollers. The port
  changes two things:
  - the spheres use the roller radius (upstream's generator uses the wheel radius);
  - the roller axes are set to 45°. Upstream's construction comes out at ~14° on a wheel this
    small, and the base then strafes at a quarter of the commanded speed.

  Opposite corners share a tilt, forming the X layout that `mecanum_drive_controller` assumes.
  `check_mecanum.py` catches a flipped tilt.
- **Free base**: the plugin publishes the free joint's pose on
  `/simulator/floating_base_state`. `mecanum_drive_controller` publishes the `odom →
  base_footprint` TF from wheel odometry.
- **Shadows**: `scene.xml` sets `<map shadowclip="6"/>`. MuJoCo's default of 1 speckles the floor
  and the base with shadow acne. The key light uses `mode="trackcom"`, so its shadow map follows
  the robot once the base drives.

## Assumptions and caveats

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
- **Passive-joint warning**: `Unable to find the actuator 'gripper_base_to_gripper_right'` is
  expected, because that joint is the passive mimic finger.
