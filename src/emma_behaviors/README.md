# emma_behaviors

A [py_trees](https://github.com/splintered-reality/py_trees) behaviour tree that sequences
emma's arm and gripper into a scripted pick and place in the MuJoCo sim. It picks a 10 mm block
off a table at a known pose (`BLOCK_START`), puts it down 8 cm to the side (`PLACE_XYZ`), checks
where it landed and returns the arm home. Planning uses `emma_manipulation`'s `ArmPlanner`
in-process; motions and the gripper run through the stock ros2_control action servers.

Perception is not in yet: the block pose is a constant written by `SetBlockPose`.

## Contents

| Path | What it is |
|---|---|
| `emma_behaviors/behaviours.py` | Custom behaviours (planning, grasp poses, attach/detach, placement check) and factories for the stock action-client and subscriber behaviours. |
| `emma_behaviors/pick_and_place.py` | The tree (`create_tree`) and the `pick_and_place` console script. |
| `launch/pick_and_place.launch.py` | `emma_simulation`'s `sim.launch.py` with `pick_scene.xml`, plus the `pick_and_place` node. Shuts everything down when the tree finishes. |
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

`ros2 launch` does not pass a node's exit code through, so `pick-check` starts the sim in the
background (log in `log/pick_check_sim.log`), runs `pick_and_place` in the foreground under a
timeout, and exits with the tree's result. `pick_and_place` exits 0 when the tree succeeds and 1
when it fails. When it finishes it prints the final tree with each behaviour's status.

A run takes about 15 s. To watch the tree live, run `pixi run py-trees-tree-watcher` in another
terminal. The graphical `py-trees-tree-viewer` (`py_trees_ros_viewer`) is not installed here.

## Tree

```
Parallel "Pick and place demo" (SuccessOnSelected [Task], synchronise=False)
├─ joints2bb                      /joint_states → joint_state (every tick)
└─ Sequence "Task"
   ├─ Wait for joints             until joint_state exists
   └─ Selector "Pick or recover"
      ├─ Retry (2 failures)
      │  └─ Sequence "Pick and place"
      │     Block pose            SetBlockPose(BLOCK_START)
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
      │     truth2bb → Check placed               sim ground truth within 20 mm of PLACE_XYZ
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
| `SetBlockPose(name, xyz, yaw)` | Writes a constant `block_pose` (stand-in for perception). |
| `CheckPlaced(name, tol=0.02)` | Compares the block's ground-truth position in `free_joints` with `PLACE_XYZ`. |
| `execute(name)` | `py_trees_ros` `FromBlackboard` client for `/arm_controller/follow_joint_trajectory`, goal from `trajectory`. |
| `open_gripper` / `close_gripper` | `FromConstant` clients for `/gripper_action_controller/gripper_cmd` at `GRIPPER_OPEN` / `GRIPPER_CLOSED`. |
| `joints_to_blackboard` | `ToBlackboard` on `/joint_states`, never cleared. |
| `free_joints_to_blackboard` | `ToBlackboard` on `/free_joint_states`, cleared on every start so it waits for a fresh sample. |

Each `PlanTo*` plans from the latest `/joint_states`, so a move starts where the arm actually
is, sag included.

## Blackboard keys

| Key | Type | Written by |
|---|---|---|
| `joint_state` | `sensor_msgs/JointState` | `joints2bb` |
| `block_pose` | `geometry_msgs/PoseStamped` in `base_link` | `SetBlockPose` |
| `grasp_candidates`, `pregrasp_candidates` | list of 4×4 TCP poses | `ComputeGraspPoses` |
| `place_candidates`, `preplace_candidates` | list of 4×4, index-aligned with the grasps | `ComputeGraspPoses` |
| `grasp_index` | `int` or `None` | `ComputeGraspPoses` (clears), the first RRT `PlanToTcp` (sets) |
| `trajectory` | `FollowJointTrajectory.Goal` | `PlanToJoints`, `PlanToTcp` |
| `free_joints` | `mujoco_ros2_control_msgs/FreeJointStateArray` | `truth2bb` |

Place candidate *i* is grasp candidate *i* moved to `PLACE_XYZ` with the same orientation, so the
block lands upright whichever tilt the pregrasp move picked.

## Failure handling

- Any behaviour failing (no IK, no plan, a rejected or aborted goal, the block not within 20 mm
  of `PLACE_XYZ`) fails the attempt. `Retry` runs the whole sequence once more, starting
  from the block pose; `ComputeGraspPoses` re-adds the block to the planning scene where
  `block_pose` says it is. That is still the constant `BLOCK_START`, so if the first attempt
  dropped or moved the block, the retry misses it and `CheckPlaced` fails again.
- After two failed attempts, `Recover` opens the gripper, detaches the block in the planning
  scene, plans home, and then returns FAILURE, so the node exits 1.
- There are no per-move timeouts; `pick-check`'s 240 s limit bounds the whole run.

## Assumptions and caveats

- **Known block pose**: `SetBlockPose` writes `BLOCK_START`, which matches the block in
  `pick_scene.xml`. Moving the block in the scene without changing the constant makes the pick
  miss.
- **Ground truth for the check**: `CheckPlaced` reads the sim's `FreeJointStatePublisherPlugin`
  (via `emma_simulation/config/mujoco_plugins.yaml`), which has no real-robot equivalent. The
  planned perception step replaces it.
- **The base is parked**: the block pose is in `base_link` and the planner assumes the base
  does not move. The plugin's poses are relative to the base body, so base drift does not
  affect the check.
- **Grasp success is not sensed**: the gripper action succeeds on reaching the goal or on
  stalling against the block. A missed grasp shows up only at `CheckPlaced`.
