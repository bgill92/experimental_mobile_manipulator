# AGENTS.md

## ROS 2 environment

ROS 2 (Lyrical Luth), colcon, and the C/C++ toolchain are provided by the pixi environment defined in `pixi.toml`. There is no system ROS install.

Run every command that builds, runs, or inspects ROS 2 inside the pixi environment:

```bash
pixi run build   # colcon build over src/ plus the upstream description packages
pixi run bash -c "source install/setup.bash && ros2 run <pkg> <node>"
pixi shell   # interactive shell with the environment activated
```

Do not call `ros2`, `colcon`, `cmake`, or the compilers directly from the host shell. Add new dependencies with `pixi add <package>` (ROS packages are named `ros-lyrical-<name>`), not with apt or pip.

## External packages

Upstream Elephant Robotics repos live in `external_packages/` as git submodules (clone with `git submodule update --init`). Only their description packages are built (see the `build` task in `pixi.toml`); the rest of those repos targets older ROS distros and will not build on Lyrical. Do not edit files inside the submodules.

## License

The project and every package under `src/` are MIT licensed (see `LICENSE`). Create new packages with `ros2 pkg create --license MIT ...` and keep `<license>MIT</license>` in each `package.xml`.
