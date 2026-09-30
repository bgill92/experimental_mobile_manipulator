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
- `gui:=true|false` starts the joint slider GUI for moving the arm (default `true`).

## License

MIT, see [LICENSE](LICENSE). The Elephant Robotics packages in `external_packages/`
keep their own licenses.
