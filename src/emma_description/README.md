# emma_description

URDF description of **emma**: an Elephant Robotics myAGV (2023, Raspberry Pi) base with a
myCobot 280 M5 arm and the parallel gripper (light) mounted on top.

## Contents

| Path | What it is |
|---|---|
| `urdf/emma.urdf.xacro` | Top-level description. Assembles base, arm and gripper, and optionally adds the `<ros2_control>` block. This is the single source of truth for both RViz and simulation. |
| `urdf/mycobot_280_m5.urdf.xacro` | myCobot 280 M5 arm, copied from `mycobot_ros2` and modified (see below). |
| `urdf/parallel_gripper.urdf.xacro` | Parallel gripper, rewritten from the ROS 1 `mycobot_ros` repo (see below). |
| `meshes/parallel_gripper/` | Gripper meshes (COLLADA, millimetres), BSD 3-Clause, see `LICENSE` there. |
| `launch/display.launch.py` | Shows the robot in RViz with joint sliders. |
| `rviz/emma.rviz` | RViz config: robot model, fixed frame `base_footprint`. |

The base description and meshes come from `myagv_description` and the arm meshes from
`mycobot_description`. Both packages are built from the submodules in `external_packages/`.

## Running

```bash
pixi run display
```

| Launch argument | Values | Default | Effect |
|---|---|---|---|
| `model` | `both`, `base`, `arm` | `both` | Whole robot, only the myAGV, or only the arm. |
| `gripper` | `parallel`, `none` | `parallel` | End effector on the arm flange. Ignored without the arm. |
| `gui` | `true`, `false` | `true` | `joint_state_publisher_gui` sliders; otherwise all joints sit at zero. |

## xacro arguments

`emma.urdf.xacro` accepts these arguments directly, for use from other launch files:

| Argument | Default | Notes |
|---|---|---|
| `model`, `gripper` | `both`, `parallel` | As above. |
| `mount_x`, `mount_y`, `mount_z`, `mount_yaw` | `0.0981`, `-0.0044`, `0.1322`, `0.0` | Arm base (`g_base`) pose relative to the AGV `base_link`. |
| `ros2_control` | `none` | `mujoco` adds a `<ros2_control>` block using `mujoco_ros2_control/MujocoSystemInterface`. Only applies to `model:=both`. |
| `mujoco_model` | `""` | MJCF scene path passed to the MuJoCo hardware plugin. |
| `headless` | `false` | Runs MuJoCo without its viewer. |
| `pids_config_file` | `""` | Wheel velocity PID gains passed to the MuJoCo hardware plugin. |

With `ros2_control:=mujoco` the block exposes:
- the six arm joints and `gripper_controller` with a `position` command and `position`/`velocity` state;
- `gripper_base_to_gripper_right` (the mimic finger) as state only, because the simulator drives it;
- the four wheel joints with a `velocity` command and `position`/`velocity` state.

## Robot structure

TF tree: `base_footprint → base_link → g_base → joint1 → … → joint6_flange → gripper_base →
{gripper_left, gripper_right}`. A mass-only `base_inertia` link and the four
`{front,rear}_{left,right}_wheel` links also hang off `base_footprint`.

| Joint | Type | Range |
|---|---|---|
| `joint2_to_joint1` | revolute | ±2.9321 rad |
| `joint3_to_joint2` | revolute | ±2.4434 rad |
| `joint4_to_joint3` | revolute | ±2.6179 rad |
| `joint5_to_joint4` | revolute | ±2.6179 rad |
| `joint6_to_joint5` | revolute | -2.7052 to 2.7925 rad |
| `joint6output_to_joint6` | revolute | ±3.14159 rad |
| `gripper_controller` | prismatic | -0.007 (closed) to 0 (open) m |
| `gripper_base_to_gripper_right` | prismatic, mimic of `gripper_controller` (×-1) | 0 to 0.007 m |
| `{front,rear}_{left,right}_wheel_joint` | continuous, axis +y | — |

The arm joint names are upstream's and read backwards: `joint2_to_joint1` rotates link
`joint2` relative to `joint1`.

## Changes from upstream

**Arm** (`mycobot_ros2` `humble`, `d42ff61`, `mycobot_280_m5.urdf`):
- The `g_base` link has no visual or collision. Upstream draws the optional G-base plate there,
  which this robot does not have.
- Inertials added. Upstream has none, and MuJoCo rejects massless moving links.
- Revolute velocity limits are 2.79 rad/s (160°/s spec) instead of upstream's 0.

**Gripper** (`mycobot_ros` `noetic`, `c0d6fbe`, `mycobot_parallel_gripper.urdf` and the
flange mount from `mycobot_280_jn_parallel_gripper.urdf`, 34 mm along the flange axis):
- Collision geometry now matches the visuals. Upstream rotated each visual by -90° about x but
  not the collision mesh, so collisions sat 90° off. That rotation cancels the +90° mount
  rotation, so both are dropped and `gripper_base` is the mesh frame.
- The right-finger joint is `gripper_base_to_gripper_right` (upstream: `gripper_base_to_gripper_left`).
- Inertials added, and the velocity limit is 0.05 m/s instead of 0.
- Upstream's `mycobot_280m5_with_gripper_parallel.urdf` is the *adaptive* gripper despite its
  name. It is not used here.

**Base** (`myagv_ros2` `galactic-JN`, `afb054b`, `myAGV.urdf`): used unchanged, since the
submodule is not edited. `emma.urdf.xacro` adds what it lacks: the mass through the
`base_inertia` link, and the four wheel links and joints, which upstream does not have.

## Assumptions and caveats

- **Arm mount pose**: taken from Elephant Robotics' product photo of the kit. The arm bolts
  straight onto the top plate, centred on the LEGO-pitch hole pattern at the +x end. Not
  measured on the real robot.
- **Masses and inertias are estimates**:
  - The arm link masses sum to the ~0.85 kg spec and the gripper is 70 g.
  - The base is 3.6 kg, plus 0.1 kg per wheel.
  - Every inertia tensor is a solid box over the link mesh's bounding box, with the centre of
    mass at the box centre. The wheels are solid cylinders instead.
  - Good enough for simulation; replace with measured or CAD values if dynamics matter.
- **Gripper travel**: the 7 mm per finger (15 mm open, 1 mm closed between fingertips) is
  upstream's value and has not been checked on the real gripper.
- **Gripper velocity limit**: 0.05 m/s is a placeholder.
- **Effort limits**: all joints use upstream's 1000, the wheels included. That is unrealistic
  but harmless; the simulated wheel motors clamp torque on their own.
- **Wheel velocity limit**: 30 rad/s is a placeholder.
- **Wheel geometry is estimated from the base mesh**: radius 0.04 m, centres at x 0.1124 /
  -0.1014 and y 0.0945 / -0.1031 (the mesh sits about 4 mm toward -y). Each wheel is a
  `continuous` joint about y, 0.1 kg, with no visual (the base mesh draws the wheels, so they
  don't visibly spin) and no collision (`emma_simulation` adds mecanum rollers). Re-measure on
  the real robot if odometry matters.
- **Mesh units differ**: arm meshes are metres, gripper meshes millimetres, base meshes inches.
  RViz reads the COLLADA `<unit>` tag so they all display correctly. Tools that ignore it, such
  as the MuJoCo converter, need rescaling (`emma_simulation` handles this).
- **The upstream robot name** inside `mycobot_280_m5.urdf.xacro` is still `firefighter`. xacro
  ignores it when the file is included.
