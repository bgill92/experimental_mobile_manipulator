# experimental_mobile_manipulator (emma)

**emma** is an experimental mobile manipulator. It uses the Elephant Robotics
[myAGV](https://github.com/elephantrobotics/myagv_ros2) (2023, Raspberry Pi) and
[myCobot 280](https://github.com/elephantrobotics/mycobot_ros2) (M5) platform for
experimentation, with the arm mounted on top of the base.

## Setup

ROS 2 Lyrical Luth and all build tools come from the [pixi](https://pixi.sh)
environment in `pixi.toml`; no system ROS install is needed.

```bash
git clone --recurse-submodules https://github.com/bgill92/experimental_mobile_manipulator.git
cd experimental_mobile_manipulator
pixi run build
```

## View the robot

```bash
pixi run bash -c "source install/setup.bash && ros2 launch emma_description display.launch.py"
```

Launch arguments:

- `model:=both|base|arm` shows the whole robot (default), only the myAGV, or only the arm.
- `gripper:=parallel|none` mounts the parallel gripper on the arm flange (default `parallel`).
- `gui:=true|false` starts the joint slider GUI for moving the arm and gripper (default `true`).

## Parallel gripper

The arm carries the Elephant Robotics
[parallel gripper (light)](https://americas.shop.elephantrobotics.com/collections/end-effectors/products/mycobot-gripper-parallel).
`mycobot_ros2` has no model of it, so it comes from the ROS 1 repo
[mycobot_ros](https://github.com/elephantrobotics/mycobot_ros) (`noetic`, `c0d6fbe`):

- Meshes: `mycobot_description/urdf/parallel_gripper/*.dae`, copied to
  `src/emma_description/meshes/parallel_gripper/` with their BSD 3-Clause license.
- Links, joints and the flange mount (34 mm along the flange axis): `mycobot_parallel_gripper.urdf`
  and `mycobot_280_jn/mycobot_280_jn_parallel_gripper.urdf`, rewritten as
  `src/emma_description/urdf/parallel_gripper.urdf.xacro`.

Changes from upstream:

- Collision geometry now matches the visuals. Upstream rotated each visual mesh by -90° about x
  but not the collision mesh, so the collision model sat 90° off. That rotation and the +90° mount
  rotation cancel out, so both are dropped and `gripper_base` is the mesh frame.
- The right-finger joint is named `gripper_base_to_gripper_right` (upstream:
  `gripper_base_to_gripper_left`).

In the joint slider GUI, `gripper_controller` moves the left finger from 0 (open, 15 mm between
the fingertips) to -0.007 m (closed, 1 mm). The right finger mirrors it through a `mimic` joint, so
it has no slider. The 7 mm travel per finger is upstream's value and has not been measured on the
real gripper. Note: upstream's `mycobot_280m5_with_gripper_parallel.urdf` is the *adaptive*
gripper despite its name.

## License

MIT, see [LICENSE](LICENSE). The Elephant Robotics packages in `external_packages/`
and the gripper meshes in `src/emma_description/meshes/parallel_gripper/` keep their own licenses.
