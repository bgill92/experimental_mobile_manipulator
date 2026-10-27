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

The upstream Elephant Robotics description packages (`myagv_description`, `mycobot_description`) are pixi source dependencies. Each has a small manifest in `external_packages/<name>/pixi.toml` that points `pixi-build-ros` at a pinned commit of the upstream repo and adds the dependencies its `package.xml` leaves out. pixi builds them into conda packages, so colcon only builds `src/`. Only the description packages are built; the rest of those repos targets older ROS distros and will not build on Lyrical. Change upstream behaviour from this repo (as `emma.urdf.xacro` does), not by patching their sources.

[beet](https://github.com/EzraBrooks/beet), a header-only C++20 behaviour tree library, is a pixi git dependency pinned at `090bfa4` (0.2.0). pixi builds it into a conda package with beet's own `pixi-build-cmake` manifest, and `src/emma_behaviors` uses it through `find_package(beet)`. To try a local beet checkout, point the `beet` entry in `pixi.toml` at it with `{ path = "..." }`. Problems found in it go into `docs/beet-feedback.md`. The Rerun C++ SDK and viewer that `emma_behaviors` logs to come from pixi (`librerun-sdk`, `rerun-sdk`).

## License

The project and every package under `src/` are MIT licensed (see `LICENSE`). Create new packages with `ros2 pkg create --license MIT ...` and keep `<license>MIT</license>` in each `package.xml`.
