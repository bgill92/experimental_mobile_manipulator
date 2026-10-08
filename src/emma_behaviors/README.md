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

**Status:** the leaves, the tree, the ROS I/O and the Rerun logging are in place and tested
without the sim. The `pick_and_place` executable, the launch file, `pick-demo` / `pick-check`
and the running instructions are still to come.

## Contents

| Path | What it is |
|---|---|
| `include/emma_behaviors/tree.hpp` | The tree (`makeTree`, `Tree`) and the `static_assert`s on its root type. |
| `include/emma_behaviors/leaves.hpp`, `src/leaves.cpp` | Edge types, error types and leaves (planning, execution, gripper, grasp poses, attach/detach, placement check). |
| `include/emma_behaviors/ros_io.hpp`, `src/ros_io.cpp` | `Io`: topic caches and action clients, `openIo`, `waitForUrdf`. |
| `include/emma_behaviors/viz.hpp`, `src/viz.cpp` | `Viz` (Rerun recording: URDF, boxes, paths, text log) and `RobotLogger` (animates the URDF from joint states). |
| `include/emma_behaviors/rerun_tree.hpp` | Tree status graph for Rerun, copied from beet's `examples/` (MIT). |
| `test/test_tree.cpp` | Tree and leaf checks against the real planner, without ROS communication or the sim. |

Run the tests with `pixi run test`. The Rerun C++ SDK (`librerun-sdk`) and viewer (`rerun-sdk`)
come from pixi; they have no ROS package names, so `package.xml` does not list them.

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
| `world/robot` | The planning URDF (with the mesh scale fixes), animated per tick from `/joint_states`. |
| `base_link/table`, `base_link/block` | Table and block boxes; the block where it was detected and where it was released. |
| `base_link/plan/<motion>` | TCP path of each executed motion (`look`, `pregrasp`, `grasp`, `lift`, ...). |
| `log` | Plan and action results, chosen grasp, placement error, recovery. |
| `tree` | Tree status graph (`rerun_tree.hpp`). |

## Differences from the py_trees version

- No parallel root: the joint-state cache is a subscription outside the tree, filled before each
  tick (beet cannot run a never-finishing branch next to the mission without changing the root's
  output type).
- No blackboard: a linear move before a grasp is chosen is a compile error (linear plans take a
  `Chosen`, which only `plan pregrasp` produces), not a runtime FAILURE.
- Recovery's handler sees only the error, so it captures the `Cell` when the tree is built.

Problems found in beet along the way are collected in
[`docs/beet-feedback.md`](../../docs/beet-feedback.md).
