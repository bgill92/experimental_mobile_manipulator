# emma_navigation

[Nav2](https://docs.nav2.org) for emma's mecanum base: a map of the living-room scene built with
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox), then map_server + AMCL for
localisation and the MPPI controller in its omnidirectional mode. The only range sensor is the 2D
lidar under the base's top plate (`/scan`, see `emma_simulation`'s README). Simulation only so
far; the topic and frame names match the myAGV's YDLidar driver so the real robot can use the same
configuration later.

## Contents

| Path | What it is |
|---|---|
| `launch/slam.launch.py` | Sim + slam_toolbox (online async, default mapper params) + RViz, for building a map. |
| `launch/navigation.launch.py` | Sim + `nav2_bringup`'s `bringup_launch.py` on a saved map + RViz. |
| `config/nav2_params.yaml` | `nav2_bringup` 1.5.2's default parameters with emma's changes (listed at the top of the file). |
| `maps/FloorPlan201.{pgm,yaml}` | Map of the MolmoSpaces living room `FloorPlan201`, origin at emma's default spawn pose. |
| `scripts/nav_check.py`, `scripts/nav_check.sh` | Headless pass/fail check behind `pixi run nav-check`. |

## Running

```bash
pixi run nav-demo    # MuJoCo viewer + RViz; send goals with RViz's "Nav2 Goal" tool
pixi run nav-check   # headless; exits 0 if emma reaches a goal past the sofa (180 s limit)
pixi run nav-map     # slam_toolbox mapping (see below)
```

All three build the workspace and write the living-room scene first, like `sim-living-room`.
Extra arguments pass through to the launch file.

| Launch argument | Default | Effect |
|---|---|---|
| `headless` | `false` | Run MuJoCo without its viewer. |
| `rviz` | `true` | Start RViz with `nav2_bringup`'s `nav2_default_view.rviz`. |
| `mujoco_model` | the composed `FloorPlan201_emma.xml` | MJCF scene. |
| `map` (`navigation.launch.py` only) | `maps/FloorPlan201.yaml` | Map for map_server. Must match the scene and spawn pose. |

`nav-check` sends one goal 2.3 m to emma's right, through the passage between the sofa and the
dining chairs, turned 90°. It passes if Nav2 reports success and the ground-truth pose from
`/simulator/floating_base_state` ends within 0.2 m of the goal. The sim log goes to
`log/nav_check_sim.log`.

## Making a map

```bash
pixi run nav-map
# second terminal, in `pixi shell` after `source install/setup.bash`:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true \
  -r cmd_vel:=/mecanum_drive_controller/reference
# when the map in RViz covers the room:
ros2 run nav2_map_server map_saver_cli -f src/emma_navigation/maps/FloorPlan201 \
  --ros-args -p use_sim_time:=true
```

slam_toolbox's map frame starts at emma's spawn pose, so AMCL's initial pose in
`config/nav2_params.yaml` is (0, 0, 0). slam_toolbox only adds a scan after 0.5 m or 0.5 rad of
motion, so the map stays sparse until emma moves. Drive slowly and keep clear of furniture: a
wheel that slips while emma is pushing something feeds slam_toolbox bad odometry, and the map
tears.

The committed map was made in `FloorPlan201` at the default spawn pose by driving a fixed
heading through out-and-back strafes along the north, east and west sides of the room. It is
derived from the MolmoSpaces scene (CC BY 4.0, AI2).

## Parameters

Changes from `nav2_bringup`'s defaults, beyond frame and topic names:

- **Mecanum base**: AMCL uses `OmniMotionModel` and MPPI the `omni` motion model with the
  `TwirlingCritic`. The velocity smoother's defaults zero `vy`, so its limits are set for all three
  axes. Speeds are capped at 0.3 m/s and 1.0 rad/s.
- **Footprint**: a rectangle around the base mesh and the arm folded at `home_q`, from x −0.155 to
  0.17 m and y −0.125 to 0.115 m. MPPI checks this footprint, not a circle. Navigating with the
  arm stretched out is not safe.
- **Costmaps**: an obstacle layer on `/scan` (no voxel layer, since there is one 2D lidar),
  inflation 0.35 m.
- **Collision monitor**: its output goes straight to `/mecanum_drive_controller/reference`, and
  its scan source accepts points from 0 m up, since the lidar sits at 8 cm.
- **Goal tolerance**: 0.1 m and 0.15 rad.

## Assumptions and caveats

- **The lidar sees only what is 8 cm above the floor**: the dining table, the coffee table top
  and chair seats show up as their legs. Nav2 could plan between the legs, under a table that the
  arm or base would hit. `nav-check`'s goal avoids that; a keepout mask over tables would fix it.
- **The map only fits the default spawn**: it is built from emma's spawn pose. Another room or
  spawn pose needs a new map.
- **AMCL does the correcting**: wheel odometry from `mecanum_drive_controller` is within a few
  percent when the wheels grip, but slips when emma pushes against furniture.
- **Frozen furniture** (see `emma_simulation`) can't be pushed aside; small loose objects can.
- **`StaticLayer: Costmap origin coordinates are not perfectly aligned`** at startup is harmless:
  the saved map's origin is not a multiple of 5 cm.
- **No hardware yet**: there is no YDLidar driver in the pixi environment, and the myAGV base
  driver targets an older ROS. Both are needed before this runs on the robot.
