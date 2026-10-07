# emma_behaviors

A [py_trees](https://github.com/splintered-reality/py_trees) behaviour tree that sequences
emma's arm, gripper and wrist camera into a pick and place in the MuJoCo sim. It looks at the
table, takes the 10 mm block's pose from `emma_perception`'s detector (`/block_pose`), picks the
block up, puts it down at `PLACE_XYZ` (8 cm to the side of `BLOCK_START`), looks again to check
where it landed and returns the arm home. Planning uses `emma_manipulation`'s `ArmPlanner`
in-process; motions and the gripper run through the stock ros2_control action servers.

## Contents

| Path | What it is |
|---|---|
| `emma_behaviors/behaviours.py` | Custom behaviours (planning, grasp poses, attach/detach, placement check) and factories for the stock action-client and subscriber behaviours. |
| `emma_behaviors/pick_and_place.py` | The tree (`create_tree`) and the `pick_and_place` console script. |
| `launch/pick_and_place.launch.py` | `emma_simulation`'s `sim.launch.py` with `pick_scene.xml`, plus `block_detector` and the `pick_and_place` node. Shuts everything down when the tree finishes. |
| `scripts/pick_check.sh` | Headless pass/fail run behind `pixi run pick-check`. |
| `test/test_behaviours.py` | Behaviour checks against the real planner, without the sim. |

## Running

```bash
pixi run pick-demo                             # MuJoCo viewer + RViz, ends when the tree does
pixi run pick-demo headless:=true rviz:=false  # no windows
pixi run pick-check                            # headless; exit 0 only if the tree succeeds (240 s limit)
```

| Launch argument | Default | Effect |
|---|---|---|
| `headless` | `false` | Run MuJoCo without its viewer. |
| `rviz` | `true` | Start RViz. |
| `tree` | `true` | Run the `pick_and_place` tree. `false` leaves the sim and detector running, for `pick-check` or for trying the detector by hand. |
| `debug` | `false` | Have `block_detector` publish its overlay on `/block_detector/debug_image`. |

`ros2 launch` does not pass a node's exit code through, so `pick-check` starts this launch with
`tree:=false` in the background (log in `log/pick_check_sim.log`), runs `pick_and_place` in the
foreground under a timeout, and exits with the tree's result. `pick_and_place` exits 0 when
the tree succeeds and 1 when it fails. When it finishes it prints the final tree with each
behaviour's status.

A run takes about 20 s of sim time (about 30 s with startup). To watch the tree live, run
`pixi run py-trees-tree-watcher` in another terminal. The graphical `py-trees-tree-viewer`
(`py_trees_ros_viewer`) is not installed here.

## Tree

```
Parallel "Pick and place demo" (SuccessOnSelected [Task], synchronise=False)
├─ joints2bb                      /joint_states → joint_state (every tick)
└─ Sequence "Task"
   ├─ Wait for joints             until joint_state exists
   └─ Selector "Pick or recover"
      ├─ Retry (2 failures)
      │  └─ Sequence "Pick and place"
      │     Plan look → Execute look              RRT to LOOK_Q (camera on the table)
      │     Settle                                Timer 1 s
      │     Detect                                Timeout 10 s: block2bb, a fresh /block_pose
      │     Grasp poses           ComputeGraspPoses
      │     Open gripper
      │     Plan pregrasp → Execute pregrasp      RRT to the first reachable candidate
      │     Plan grasp → Execute grasp            straight line
      │     Close gripper → Attach block
      │     Plan lift → Execute lift              straight line back to pregrasp
      │     Plan preplace → Execute preplace      RRT
      │     Plan place → Execute place            straight line
      │     Release → Detach block
      │     Plan retreat → Execute retreat        straight line back to preplace
      │     Plan check look → Execute check look  back to LOOK_Q
      │     Check settle → Check detect           a fresh /block_pose, as above
      │     Check placed                          within 20 mm of PLACE_XYZ
      │     Plan home → Execute home
      └─ Sequence "Recover"
         Open → Detach → Plan home → Execute home → Give up (FAILURE)
```

All sequences have memory, so a behaviour that succeeded is not re-run while a later one is
running. The planning behaviours run synchronously inside a tick (RRT budget 2 s); the tree
ticks at 5 Hz.

## Behaviours

| Behaviour | Does |
|---|---|
| `PlanToJoints(name, planner, q_goal)` | RRT from the current joints to `q_goal`; writes `trajectory`. |
| `PlanToTcp(name, planner, key, linear)` | Plans the TCP to `key` (one 4×4 or a candidate list): straight line if `linear`, else RRT. For a list it uses `grasp_index` if set; otherwise an RRT move tries the candidates in order and sets `grasp_index`. |
| `ComputeGraspPoses(name, planner)` | From `block_pose`: grasp, pregrasp, place and preplace candidates, the block box in the planning scene, and `grasp_index` cleared. |
| `AttachBlock` / `DetachBlock` | `planner.attach/detach('block')` at the current joints. `DetachBlock` succeeds without a block. |
| `CheckPlaced(name, tol=0.02)` | Compares the detected block position in `block_pose` with `PLACE_XYZ`. |
| `execute(name)` | `py_trees_ros` `FromBlackboard` client for `/arm_controller/follow_joint_trajectory`, goal from `trajectory`. |
| `open_gripper` / `close_gripper` | `FromConstant` clients for `/gripper_action_controller/gripper_cmd` at `GRIPPER_OPEN` / `GRIPPER_CLOSED`. |
| `joints_to_blackboard` | `ToBlackboard` on `/joint_states`, never cleared. |
| `block_to_blackboard(name)` | `ToBlackboard` on `/block_pose`, cleared on every start, so it stays RUNNING until a detection arrives after it starts: one fresh sample per visit. |

Each `PlanTo*` plans from the latest `/joint_states`, so a move starts where the arm actually
is, sag included.

## Blackboard keys

| Key | Type | Written by |
|---|---|---|
| `joint_state` | `sensor_msgs/JointState` | `joints2bb` |
| `block_pose` | `geometry_msgs/PoseStamped` in `base_link` | `block2bb`, `check block2bb` |
| `grasp_candidates`, `pregrasp_candidates` | list of 4×4 TCP poses | `ComputeGraspPoses` |
| `place_candidates`, `preplace_candidates` | list of 4×4, index-aligned with the grasps | `ComputeGraspPoses` |
| `grasp_index` | `int` or `None` | `ComputeGraspPoses` (clears), the first RRT `PlanToTcp` (sets) |
| `trajectory` | `FollowJointTrajectory.Goal` | `PlanToJoints`, `PlanToTcp` |

Place candidate *i* is grasp candidate *i* moved to `PLACE_XYZ` with the same orientation, so the
block lands upright whichever tilt the pregrasp move picked.

## Failure handling

- Any behaviour failing (no IK, no plan, a rejected or aborted goal, no detection within 10 s,
  the block not within 20 mm of `PLACE_XYZ`) fails the attempt. `Retry` runs the whole
  sequence once more, starting with a fresh look, so a block the first attempt dropped or
  pushed is picked from where it now lies (if it is still in view and reachable);
  `ComputeGraspPoses` moves the block in the planning scene to the new pose.
  The retry does not open the gripper first, though: after a failure mid-carry it looks with
  the block still in the fingers, and the open before the pregrasp drops it from there.
- After two failed attempts, `Recover` opens the gripper, detaches the block in the planning
  scene, plans home, and then returns FAILURE, so the node exits 1.
- There are no per-move timeouts; `pick-check`'s 240 s limit bounds the whole run.

## Assumptions and caveats

- **Perceived block pose**: the block can sit anywhere the camera sees it from `LOOK_Q` and
  the arm can reach. In the sim, blocks at (0.28, 0.03) turned 25°, (0.27, -0.025) turned 15°,
  (0.26, -0.03) turned -20° and (0.24, 0.02) turned -30° were picked and placed. Closer to the
  robot than about x = 0.23 m the straight-line grasp move finds no solution (with or without
  the camera), so the pick fails there.
- **One sample per look**: the tree takes the first detection after a 1 s settle and does not
  average. In the sim the detection is within about 1 mm of the ground truth at `LOOK_Q`.
- **The base is parked**: the block pose is in `base_link` and the planner assumes the base
  does not move between the look and the place.
- **Grasp success is not sensed**: the gripper action succeeds on reaching the goal or on
  stalling against the block. A missed grasp shows up only at `CheckPlaced`.
- **The check uses the same detector**: `CheckPlaced` trusts `/block_pose`. To compare with the
  sim's ground truth, `/free_joint_states` still publishes the block's pose relative to
  `base_footprint` (which coincides with `base_link`).
