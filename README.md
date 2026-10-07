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

- [`emma_description`](src/emma_description/README.md): URDF of the robot (myAGV, myCobot 280 M5
  and parallel gripper), RViz display launch, and the changes made to the upstream models.
- [`emma_simulation`](src/emma_simulation/README.md): MuJoCo simulation through
  mujoco_ros2_control, with arm, gripper and mecanum base controllers.
- [`emma_manipulation`](src/emma_manipulation/README.md): arm motion planning with
  [roboplan](https://github.com/open-planning/roboplan) (IK, RRT, straight-line moves) and a
  `move_arm` command-line tool.

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

Launch arguments, example action goals, assumptions and caveats are in each package's README.

## Test

```bash
pixi run test   # unit tests and linters
```

## License

MIT, see [LICENSE](LICENSE). The Elephant Robotics packages in `external_packages/`
and the gripper meshes in `src/emma_description/meshes/parallel_gripper/` keep their own licenses.
MolmoSpaces scenes are downloaded on demand, not included, and are CC BY 4.0.
