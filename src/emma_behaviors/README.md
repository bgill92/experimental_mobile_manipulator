# emma_behaviors

emma's pick and place as a typed behaviour tree on [beet](https://github.com/EzraBrooks/beet), a
header-only C++20 library (git submodule at `external_packages/beet`, pinned at `436617cd`). The
tree looks at the table, takes the 10 mm block's pose from `emma_perception`'s detector
(`/block_pose`), picks the block up, puts it down at `kPlaceXyz` (8 cm to the side of
`kBlockStart`), looks again to check where it landed and returns the arm home. Planning uses
`emma_manipulation`'s `ArmPlanner` in-process; motions and the gripper run through the stock
ros2_control action servers. The tree and the scene are logged to [Rerun](https://rerun.io).

In beet, data flows along the tree's edges: each node is a function from an input type to an
output type or an error, and the compiler checks that each node accepts what the previous one
produces. There is no blackboard. The root's type lists every failure the tree can produce.

## Contents

| Path | What it is |
|---|---|
| `include/emma_behaviors/tree.hpp` | The tree (`makeTree`, `Tree`) and the `static_assert`s on its root type. |
| `include/emma_behaviors/leaves.hpp`, `src/leaves.cpp` | Edge types, error types and leaves (planning, execution, gripper, grasp poses, attach/detach, placement check). |
| `include/emma_behaviors/ros_io.hpp`, `src/ros_io.cpp` | `Io`: topic caches and action clients, `openIo`, `waitForUrdf`. |
| `include/emma_behaviors/viz.hpp`, `src/viz.cpp` | `Viz` (Rerun recording: URDF, boxes, paths, text log) and `RobotLogger` (animates the URDF from joint states). |
| `include/emma_behaviors/rerun_tree.hpp` | Tree status graph for Rerun, copied from beet's `examples/` (MIT). |
| `src/pick_and_place.cpp` | The `pick_and_place` executable: the 5 Hz loop that ticks the tree and logs to Rerun. |
| `launch/pick_and_place.launch.py` | `emma_simulation`'s `sim.launch.py` with `pick_scene.xml`, plus `block_detector` and the `pick_and_place` node. Shuts everything down when the tree finishes. |
| `scripts/pick_check.sh` | Headless pass/fail run behind `pixi run pick-check`. |
| `test/test_tree.cpp` | Tree and leaf checks against the real planner, without ROS communication or the sim. |

Run the tests with `pixi run test`. The Rerun C++ SDK (`librerun-sdk`) and viewer (`rerun-sdk`)
come from pixi; they have no ROS package names, so `package.xml` does not list them.

## Running

```bash
pixi run pick-demo                             # MuJoCo viewer + RViz + Rerun viewer, ends when the tree does
pixi run pick-demo headless:=true rviz:=false  # no sim windows; the Rerun viewer still opens
pixi run pick-check                            # headless; exit 0 only if the tree succeeds (240 s limit)
pixi run rerun log/pick_check.rrd              # replay the last pick-check in the Rerun viewer
```

| Launch argument | Default | Effect |
|---|---|---|
| `headless` | `false` | Run MuJoCo without its viewer. |
| `rviz` | `true` | Start RViz. |
| `tree` | `true` | Run the `pick_and_place` tree. `false` leaves the sim and detector running, for `pick-check` or for trying the detector by hand. |
| `debug` | `false` | Have `block_detector` publish its overlay on `/block_detector/debug_image`. |

`pick_and_place` spawns a Rerun viewer unless it is given `--save FILE.rrd`, in which case it
writes the recording to that file and opens no window
(`ros2 run emma_behaviors pick_and_place --save run.rrd --ros-args -p use_sim_time:=true`). If
the viewer cannot be spawned it prints a warning and runs without one.

`ros2 launch` does not pass a node's exit code through, so `pick-check` starts this launch with
`tree:=false` in the background (log in `log/pick_check_sim.log`), runs `pick_and_place --save
log/pick_check.rrd` in the foreground under a timeout, and exits with the tree's result.

Exit codes of `pick_and_place`: 0 when the tree succeeds, 1 when it fails (`GaveUp`), cannot get
`/robot_description` within 30 s or an action server within 10 s, or is interrupted; 2 for an
unknown command-line argument. When it finishes it prints the final tree, one node per line
indented by depth, with each node's status (unnamed nodes show their kind, e.g. `sequence`, and
the leaf under `detect` shows as `leaf`).

A run takes about 45 s of wall time after startup (about 220 ticks).

## Tree

Edge types in brackets: each node's output is the next node's input.

```
Sequence                                          [Cell -> Cell], fails only with GaveUp
├─ wait for joints                                until a joint state with all arm joints
└─ Fallback
   ├─ retry (2 attempts)
   │  └─ pick and place (Sequence)                [Cell -> Cell]
   │     plan look → execute look                 RRT to kLookQ (offloaded)    [Cell -> Planned<Cell> -> Cell]
   │     settle                                   5 ticks
   │     detect                                   timeout 50 ticks: a fresh /block_pose   [-> Looked]
   │     grasp poses                              candidates + block box       [Looked -> Grasps]
   │     open gripper
   │     plan pregrasp → execute pregrasp         RRT to the first reachable candidate (offloaded)  [Grasps -> Chosen]
   │     plan grasp → execute grasp               straight line                [Chosen -> Chosen]
   │     close gripper → attach block
   │     plan lift → execute lift                 straight line back to pregrasp
   │     plan preplace → execute preplace         RRT (offloaded)
   │     plan place → execute place               straight line
   │     release → detach block
   │     plan retreat → execute retreat           straight line back to preplace
   │     plan look → execute look → settle → detect   as above                [Chosen -> Looked]
   │     check placed                             within 20 mm of kPlaceXyz    [Looked -> Cell]
   │     plan home → execute home                 RRT (offloaded)
   └─ recover (Sequence)
      recover<ActionFailed, NoPlan>(open → detach → plan home → execute home)
      give up                                     always fails with GaveUp
```

## Error types

| Error | Raised by | Handled by |
|---|---|---|
| `NoPlan` | planning leaves (no IK, no path, no joint state) | `retry`, then the fallback; inside recovery by `recover<ActionFailed, NoPlan>` |
| `ActionFailed` | `execute`, gripper leaves (rejected, aborted, cancelled) | as `NoPlan` |
| `beet::Timeout` | `detect` (no fresh `/block_pose` within 50 ticks) | `retry`, then the fallback |
| `NotPlaced` | `check placed` | `retry`, then the fallback |
| `GaveUp` | `give up` | nothing: the root's only error, so the run fails on purpose |

## Ticks and threads

The tree ticks at 5 Hz (`kTickPeriod`); beet has tick-count timeouts only, so settling is 5 ticks
(1 s) and the detection timeout 50 ticks (10 s). RRT plans run in `beet::offload` on their own
thread while the tree keeps ticking; linear plans and grasp poses take milliseconds and run in the
tick. Every planner leaf is in one sequence, so the non-thread-safe `ArmPlanner` is never used by
two threads at once. ROS callbacks run in the main loop before each tick and touch only `Io`; the
joint state in `Io` has a mutex because offloaded plans read it.

## Rerun entities

| Entity | Content |
|---|---|
| `world/robot` | The planning URDF (with the mesh scale fixes), animated per tick from `/joint_states` (`world/robot/joints/<joint>`). The right finger stays closed in the viewer because the planning URDF drops its mimic joint. |
| `base_link/table`, `base_link/block` | Table and block boxes; the block where it was detected and where it was released. |
| `base_link/plan/<motion>` | TCP path of each executed motion (`look`, `pregrasp`, `grasp`, `lift`, ...). |
| `log` | Plan and action results, chosen grasp, placement error, recovery. |
| `tree` | Tree status graph (`rerun_tree.hpp`). |

The planning URDF is logged rather than `/robot_description` because Rerun 0.38's COLLADA importer
ignores the `<unit>` tag (it applies `<up_axis>` only; see `re_renderer/src/importer/dae.rs`), just
as coal and assimp do in the planner, so the gripper (millimetres) and myAGV (inches) meshes need
the URDF `scale` the rewrite adds. Without it they would show 1000× and 39× too large.

Timelines: `tick` (one per tree tick) and `time` (wall seconds since the loop started). Events
logged from offloaded planning threads carry only Rerun's `log_time`.

## Differences from the py_trees version

- No parallel root: the joint-state cache is a subscription outside the tree, filled before each
  tick (beet cannot run a never-finishing branch next to the mission without changing the root's
  output type).
- No blackboard: a linear move before a grasp is chosen is a compile error (linear plans take a
  `Chosen`, which only `plan pregrasp` produces), not a runtime FAILURE.
- Recovery's handler sees only the error, so it captures the `Cell` when the tree is built.

## Assumptions and caveats

- **Perceived block pose**: the block can sit anywhere the camera sees it from `kLookQ` and the arm
  can reach. With the py_trees version, blocks at (0.28, 0.03) turned 25°, (0.27, -0.025) turned
  15°, (0.26, -0.03) turned -20° and (0.24, 0.02) turned -30° were picked and placed; the planner
  and grasp poses are ports of the same code. Closer to the robot than about x = 0.23 m the
  straight-line grasp move finds no solution, so the pick fails there.
- **One sample per look**: the tree takes the first detection after settling and does not
  average. In the sim the detection is within about 1 mm of the ground truth at `kLookQ`.
- **The base is parked**: the block pose is in `base_link` and the planner assumes the base does
  not move between the look and the place.
- **Grasp success is not sensed**: the gripper action succeeds on reaching the goal or on stalling
  against the block. A missed grasp shows up only at `check placed`.
- **The check uses the same detector**: `check placed` trusts `/block_pose`. To compare with the
  sim's ground truth, `/free_joint_states` still publishes the block's pose relative to
  `base_footprint` (which coincides with `base_link`).
- **Retry keeps the gripper as it was**: after a failure mid-carry the retry looks with the block
  still in the fingers, and the open before the pregrasp drops it from there.
- **No per-move timeouts**: only `detect` has one; `pick-check`'s 240 s limit bounds the whole run.

Problems found in beet along the way are collected in
[`docs/beet-feedback.md`](../../docs/beet-feedback.md).
