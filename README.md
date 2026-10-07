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

## Packages

- [`emma_description`](src/emma_description/README.md): URDF of the robot (myAGV, myCobot 280 M5,
  parallel gripper and a wrist RGB-D camera), RViz display launch, and the changes made to the
  upstream models.
- [`emma_simulation`](src/emma_simulation/README.md): MuJoCo simulation through
  mujoco_ros2_control, with arm, gripper and mecanum base controllers and the wrist camera's
  colour and depth images.
- [`emma_manipulation`](src/emma_manipulation/README.md): arm motion planning with
  [roboplan](https://github.com/open-planning/roboplan) (IK, RRT, straight-line moves) and a
  `move_arm` command-line tool.
- [`emma_perception`](src/emma_perception/README.md): finds the red block in the wrist camera's
  colour and depth images and publishes its pose on `/block_pose`.
- [`emma_behaviors`](src/emma_behaviors/README.md): [py_trees](https://github.com/splintered-reality/py_trees)
  behaviour tree for a pick and place of the block in the sim, using the detected pose.

## View the robot

```bash
pixi run display
```

## Simulate the robot

```bash
pixi run sim        # MuJoCo viewer + RViz; add headless:=true rviz:=false to run without windows
pixi run gen-mjcf   # regenerate the MuJoCo model after URDF changes
```

To put emma in a furnished living room from [MolmoSpaces](https://github.com/allenai/molmospaces):

```bash
pixi run molmospaces-scene   # downloads the room once, writes the combined scene
pixi run sim mujoco_model:=$PWD/src/emma_simulation/mujoco/molmospaces/scenes/ithor/FloorPlan201_emma.xml
```

With the sim running, move the arm by planning to a TCP pose or back home:

```bash
pixi run bash -c "source install/setup.bash && ros2 run emma_manipulation move_arm --pose 0.25 0 0.20 0 3.1416 0"
pixi run bash -c "source install/setup.bash && ros2 run emma_manipulation move_arm --home"
```

## Pick and place in sim

```bash
pixi run pick-demo    # table + block scene: look, detect, pick, place 8 cm over, look again, home
pixi run pick-check   # the same headless; exits 0 only if the camera sees the block at the target
```

Launch arguments, example action goals, assumptions and caveats are in each package's README.

## Test

```bash
pixi run test   # unit tests and mypy
```

## License

MIT, see [LICENSE](LICENSE). The Elephant Robotics packages in `external_packages/`
and the gripper meshes in `src/emma_description/meshes/parallel_gripper/` keep their own licenses.
MolmoSpaces scenes are downloaded on demand, not included, and are CC BY 4.0.
