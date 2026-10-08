# emma: pick-and-place on beet (C++ typed behavior trees) instead of py_trees

Self-contained implementation plan. An agent starting from this file needs nothing else from the
conversation that produced it. Read `AGENTS.md` first (every ROS/colcon/cmake command runs through
`pixi run ...`; no system ROS).

## Status / handoff (as of 2026-10-08)

Done:
- PRs #2–#5 (old py_trees stack) closed with a comment; branches `pick-place/m1-manipulation`,
  `m2-behavior-tree`, `m3-perception`, `m4-docs` kept as reference and cherry-pick source.
- Branch `beet/m1-model-sim` created off `origin/main` (`a3cdc73`).
- Cherry-picked and committed on it: `47b85fd` (tcp link) → `c9c6ba9`; `e99296f` (wrist camera,
  `src/emma_description` hunks only, the `emma_manipulation` Python hunks dropped) → `4076699`
  (its message still says "and a look pose for it"; amend to "Add a wrist RGB-D camera to emma" when
  convenient); `69ba163` (finger collisions + pick scene); `0cc957b` (camera render + base no-roll).
- `pixi add ros-lyrical-mujoco-ros2-control-plugins ros-lyrical-mujoco-ros2-control-msgs` done.
- `pixi run build` passes. Headless smoke test of `pick_scene.xml` passes: topics
  `/wrist_camera/color/{image_raw,camera_info}`, `/wrist_camera/depth/image_raw` at 5 Hz,
  `/free_joint_states`, `/joint_states`, `/clock` present.

Uncommitted in the working tree (M1 remainder, ready to commit):
- `.gitignore` (+ `__pycache__/`, `*.pyc`, `MUJOCO_LOG.TXT`), `pixi.toml` + `pixi.lock` (the two
  plugin deps), `src/emma_description/README.md` and `src/emma_simulation/README.md` (rewraps taken
  from `pick-place/m4-docs`), this file.

Next: commit the M1 remainder (two or three commits: deps+gitignore, README rewraps, plan doc), push,
open PR `beet/m1-model-sim → main`, then M2.

## Context

The pick-and-place plan (`docs/plans/2026-10-06-pick-and-place-plan.md`, only on the old branches)
was fully implemented with py_trees on the unmerged stack above. A colleague published **beet**
(https://github.com/EzraBrooks/beet, MIT, pre-alpha): header-only C++20 behavior trees where data
flows along typed edges, failures are part of the node type, and `recover<E>` removes handled errors
so the root type proves what the tree can fail with. Its `examples/roboplan_ur5.cpp` plans with
roboplan's C++ RRT on an offloaded thread and logs tree + scene to Rerun. We want that stack
(beet + roboplan C++ + Rerun) as emma's behavior layer.

Decisions (confirmed with the user):
- **All new work branches off `main`** as a fresh milestone stack of stacked PRs
  (`beet/m1-model-sim` → `beet/m2-perception` → `beet/m3-manipulation` → `beet/m4-behaviors`).
- **Planner moves to C++** inside `emma_manipulation` (ament_cmake). No Python planner; `move_arm`
  re-done in C++.
- **Rerun** is a required dep: tree status graph per tick + scene (URDF animated from
  `/joint_states`, planned EE paths, table/block boxes, text log).
- Sim scene, wrist camera, `emma_perception` (Python, `/block_pose`) are reused from the old
  branches by cherry-pick; they do not touch the planner.
- READMEs updated inside each milestone (no separate docs milestone). Audit root + package READMEs
  after every change.
- Problems found in beet go into `docs/beet-feedback.md` for its author (see below).

## Verified environment facts

| Item | Status |
|---|---|
| beet | header-only, C++20 coroutines + `tl::expected`; not on conda-forge; pin `436617cd2fd3abc5811223b63977b2ec74800c27` (2026-10-07). CMake: `find_package(Threads)` + `FetchContent(tl-expected, FIND_PACKAGE_ARGS 1.1)` → resolves offline against the installed `share/cmake/tl-expected/tl-expected-config.cmake`. Tests/examples default OFF when not top level |
| GCC in pixi env | 15.3 (`x86_64-conda-linux-gnu-g++`) |
| roboplan C++ 0.7 | installed via `ros-lyrical-roboplan`; headers `include/roboplan/core/{scene,scene_context,types,path_utils,pose_utils,geometry_wrappers}.hpp`, `roboplan_rrt/rrt.hpp`, `roboplan_simple_ik/simple_ik.hpp`, `roboplan_toppra/toppra.hpp`; CMake targets `roboplan::roboplan` (`find_package(roboplan_core)`), `roboplan_rrt::roboplan_rrt`, `roboplan_simple_ik::roboplan_simple_ik`, `roboplan_toppra::roboplan_toppra`, `roboplan::tl_expected` |
| roboplan API parity with `planner.py` | `loadUrdfSceneDescriptionFromXml(xml, package_paths)`, `Scene(name, desc)`, `addGroupFromChain`, `allowAdjacentLinkCollisions`, `setCollisions(vector<pair<string,string>>, bool)`, `setRngSeed`, `getCurrentJointPositions`, `getJointPositionIndices(vector<string>) -> VectorXi`, `clampToValidConfiguration`, `hasCollisions(q)`, `forwardKinematics(q, frame, base)` plus the documented thread-safe `forwardKinematics(pinocchio::Data&, q, frame, base)`, `addBoxGeometry(name, parent, Box, Matrix4d, Vector4d) -> tl::expected<void,string>`, `removeGeometry`, `SimpleIk(scene, SimpleIkOptions{group_name,max_iters,max_time,max_restarts,check_collisions})::solveIk(CartesianConfiguration, JointConfiguration, JointConfiguration&) -> bool`, `RRT(scene, RRTOptions{group_name,max_connection_distance,collision_check_step_size,max_planning_time,rrt_connect})::plan(start, goal) -> tl::expected<JointPath,string>` (no exceptions; the Python `RuntimeError` was the binding's translation), `PathShortcutter(scene, PathShortcuttingOptions{group_name,max_step_size,max_iters,seed})::shortcut(JointPath)`, `poseError(Matrix4d, Matrix4d) -> pair<double,double>` (pose_utils.hpp). Types: `JointConfiguration{joint_names, positions}`, `CartesianConfiguration{base_frame, tip_frame, tform}`, `JointPath{joint_names, positions}` |
| tinyxml2 11 (`tinyxml2::tinyxml2`, roboplan already links it), cpp-expected 1.3.1, rclcpp, rclcpp_action, control_msgs (`control_msgs/action/{follow_joint_trajectory,parallel_gripper_command}.hpp`), ament_index_cpp, eigen3, pinocchio, ament_cmake_gtest, GTest, xacro CMake (`share/xacro/cmake/xacro-extras.cmake`: `xacro_add_xacro_file(<in> <out>)`) | installed |
| Rerun | `librerun-sdk` / `rerun-sdk` 0.38.1 on conda-forge (not yet deps). `rerun-sdk` (Python) ships the `rerun` viewer binary that `RecordingStream::spawn()` execs; beet's pixi.toml pairs the two |
| py_trees deps | not on `main`; nothing to remove |

## beet semantics that shape the port

- `Node<In, Out, Err> = In -> Task<Result<Out, Err>>`; `Result` is `tl::expected`. No blackboard:
  state rides along node outputs in a `Cell`-like struct (as the example).
- Leaves: `Out(In)`, `Result<Out,E>(In)`, or coroutine `Task<Result<Out,E>>(In)` yielding with
  `co_await beet::running` once per tick. Halt = coroutine destroyed; cleanup in destructors or
  `finally(node, f)`. Composites/decorators: `sequence`, `fallback`, `parallel_all/any/n<K>`,
  `recover<E...>(node, handler)`, `retry(n, node)`, `repeat(n, node)`, `timeout_ticks(n, node)`
  (fails with `beet::Timeout`), `condition(pred)`, `named<"x">(node)` (label lives in the type).
- `beet::offload(fn)` runs a plain function on its own thread; optional trailing `std::stop_token`.
  A halted job runs to completion detached; result dropped.
- `beet::Channel<Req, Reply>` request/reply between parallel branches (not needed here).
- Tracing: `beet::StatusTable<Tree>` observer (`beet/observe.hpp`, not included by `beet.hpp`),
  constexpr `beet::tree_info<Tree>` (`node_info{kind,label,input,output,error,parent}`, depth-first
  IDs, root 0, `beet::no_parent`), `beet::Runner{tree, input, observer}`; `runner.tick()` returns
  `Status{Idle,Running,Success,Failure}`; `runner.result()` after it finished.
- Gaps (from beet's README): recover handlers see only the error (capture context at build time);
  parallel children need one input type and `parallel_any` reports `variant<Out, never>` for a
  never-finishing branch; same-type errors merge (use distinct error structs); decorator params are
  construction-time; no wall timers (use tick counts); offloaded functions' thread safety is unchecked.
- Reference sources: `external_packages/beet/examples/roboplan_ur5.cpp` (Cell struct, offload,
  controller coroutine, Rerun scene logging) and `examples/rerun_tree.hpp` (GraphLogger).

## Milestones (branches off `main`, stacked PRs)

### M1 `beet/m1-model-sim` — robot model and sim ground work (cherry-picks, no planner)
See Status above; cherry-picks are done. Remaining:
- Commit `.gitignore`, `pixi.toml`/`pixi.lock`, README rewraps, this plan doc.
- `git push -u origin beet/m1-model-sim`; `gh pr create --base main`.
Verify (done once; re-run after commits if anything changed): `pixi run build`;
`pixi run sim headless:=true rviz:=false mujoco_model:=$PWD/install/emma_simulation/share/emma_simulation/mujoco/pick_scene.xml`
and `ros2 topic hz /wrist_camera/color/image_raw` ≈ 5 Hz, `/free_joint_states` lists `block`.

### M2 `beet/m2-perception` — block detector (cherry-pick)
- `git checkout -b beet/m2-perception beet/m1-model-sim`.
- `git cherry-pick 08bfe8d` (`src/emma_perception`, Python ament_python). It also touched
  `pixi.toml`/`pixi.lock`; resolve by keeping ours and running
  `pixi add "ros-lyrical-cv-bridge>=4.1.0,<5"` instead.
- Drop its ament template lint tests (`test/test_copyright.py`, `test_flake8.py`, `test_pep257.py`,
  `test_xmllint.py`) and the matching `<test_depend>`s, as `a97b097` did for emma_manipulation; keep
  `test_block_detector.py` and `test_mypy.py`.
- pixi `test` task: `colcon test --packages-select emma_perception && colcon test-result --verbose`.
- README: `src/emma_perception/README.md` from `pick-place/m4-docs` (check it against the code);
  root `README.md` package list.
Verify: `pixi run build && pixi run test`; with the M1 sim up (`pick_scene.xml`) and
`ros2 run emma_perception block_detector --ros-args -p use_sim_time:=true`, `ros2 topic echo --once
/block_pose` is near `(0.25, 0, 0.155)` in `base_link` (the arm must be at a pose where the camera
sees the table; from `HOME_Q` it does not, so this topic check is best done in M4 via the tree).

### M3 `beet/m3-manipulation` — C++ planner library and `move_arm`
`git checkout -b beet/m3-manipulation beet/m2-perception`. `src/emma_manipulation` (ament_cmake),
written fresh with the old Python as reference:
`git show pick-place/m4-docs:src/emma_manipulation/emma_manipulation/{planner,grasp,constants,move_arm}.py`
and `git show pick-place/m4-docs:src/emma_manipulation/test/{test_planner,test_grasp}.py`.
```
CMakeLists.txt package.xml LICENSE README.md
include/emma_manipulation/{constants,grasp,planner,trajectory}.hpp
src/{grasp,planner,trajectory,move_arm}.cpp
test/{test_grasp,test_planner}.cpp
```
- Library `emma_manipulation` (SHARED, `target_compile_features(cxx_std_20)`): links
  `roboplan::roboplan roboplan_rrt::roboplan_rrt roboplan_simple_ik::roboplan_simple_ik
  tinyxml2::tinyxml2 Eigen3::Eigen` + `ament_target_dependencies(sensor_msgs trajectory_msgs
  builtin_interfaces ament_index_cpp)`; `ament_export_targets(... HAS_LIBRARY_TARGET)` +
  `ament_export_dependencies(roboplan_core roboplan_rrt roboplan_simple_ik tinyxml2 Eigen3
  sensor_msgs trajectory_msgs ament_index_cpp)` so `emma_behaviors` gets transitive includes.
- `constants.hpp` (namespace `emma_manipulation`): `inline const std::vector<std::string> kArmJoints`
  (roboplan wants `vector<string>`), `inline const Eigen::VectorXd kHomeQ`, `kLookQ`,
  `inline constexpr std::string_view kBaseFrame = "base_link"`, `kCameraFrame =
  "wrist_camera_color_optical_frame"`, `kTcpFrame = "tcp"`, `inline const Eigen::Vector3d
  kTableCenter, kTableHalfSize, kBlockStart, kPlaceXyz`, `inline constexpr double kBlockSize,
  kPreOffset, kGripperOpen = 0.0, kGripperClosed = -0.007`, `inline constexpr std::array<double, 3>
  kGraspTilts`. Values from the old `constants.py`: `ARM_JOINTS = [joint2_to_joint1,
  joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6]`,
  `HOME_Q = [0, 2.259, -2.505, 0.3, 0, 0]`, `LOOK_Q = [0.862, 0.771, -1.533, -0.695, -0.133, 0.855]`,
  table centre (0.30, 0, 0.075) half-size (0.10, 0.15, 0.075), block 0.010 at (0.25, 0, 0.155),
  place (0.25, 0.08, 0.155), pre-offset 0.05, tilts 0 / 20° / 35°.
- `grasp.hpp`: `[[nodiscard]] std::vector<Eigen::Matrix4d> graspCandidates(const Eigen::Vector3d&
  block_xyz, double block_yaw, const Eigen::Vector2d& arm_base_xy, std::span<const double> tilts =
  kGraspTilts)` (fingers close along the block axis most perpendicular to the base→block line;
  approach z down, tilted toward the reach; each tilt twice with fingers swapped);
  `[[nodiscard]] Eigen::Matrix4d offset(const Eigen::Matrix4d& tform, double dist)` (back off along
  the pose's own -z).
- `trajectory.hpp`: `[[nodiscard]] trajectory_msgs::msg::JointTrajectory toJointTrajectory(const
  std::vector<Eigen::VectorXd>& path, double v_max = 0.8, double min_segment_time = 0.1)` (drops the
  first waypoint; per segment `t += max(max|dq| / v_max, min_segment_time)`;
  `builtin_interfaces::msg::Duration` from `std::llround(t * 1e9)` split into sec/nanosec).
- `planner.hpp`:
  - `struct PlanningUrdf { std::string xml; std::vector<std::string> link_names; };`
    `[[nodiscard]] tl::expected<PlanningUrdf, std::string> rewriteUrdfForPlanning(const std::string&
    urdf_xml)` with tinyxml2: remove every `<mimic>` under `<joint>`; for each `<link>` with a
    `<visual>` whose `<geometry><mesh filename>` contains `myagv_` and no `<collision>`, add a
    `<collision>` holding `DeepClone`s of the visual's `<origin>` and `<geometry>`; set
    `scale="0.001 0.001 0.001"` on every `<mesh>` whose filename contains `parallel_gripper/` and
    `scale="0.0254 0.0254 0.0254"` for `myagv_` (coal/assimp ignore the COLLADA `<unit>` tag).
  - `[[nodiscard]] std::vector<std::filesystem::path> meshPackagePaths()`: parents of
    `ament_index_cpp::get_package_share_directory` for `emma_description`, `mycobot_description`,
    `myagv_description` (`package://pkg/...` resolves against the directory holding the share dir).
  - `class ArmPlanner`:
    ```cpp
    struct Options { std::string base_frame{kBaseFrame}; std::string tip_frame{kTcpFrame};
                     double max_planning_time = 2.0; std::optional<unsigned> seed; };
    /// @throws std::runtime_error if the URDF cannot be rewritten/loaded or the group cannot be built.
    /// @warning Not thread-safe: the scene scratch and box bookkeeping are shared and mutated.
    explicit ArmPlanner(const std::string& urdf_xml, const Options& options = {});
    [[nodiscard]] static tl::expected<Eigen::VectorXd, std::string> armPositions(const sensor_msgs::msg::JointState&);  // kArmJoints order; error lists missing joints
    [[nodiscard]] Eigen::VectorXd fullQ(const Eigen::VectorXd& q) const;      // scene full vector with arm slots replaced
    [[nodiscard]] Eigen::VectorXd clamp(const Eigen::VectorXd& q) const;      // clampToValidConfiguration
    [[nodiscard]] bool hasCollisions(const Eigen::VectorXd& q) const;
    [[nodiscard]] Eigen::Matrix4d fk(const Eigen::VectorXd& q, std::string_view frame = {}) const;  // empty = tip, in base_frame
    [[nodiscard]] std::optional<Eigen::VectorXd> ik(const Eigen::Matrix4d& tform, const Eigen::VectorXd& q_seed, bool local = false);
    [[nodiscard]] tl::expected<std::vector<Eigen::VectorXd>, std::string> planJoint(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal);
    struct PlanToAny { std::vector<Eigen::VectorXd> path; std::size_t index; };
    [[nodiscard]] tl::expected<PlanToAny, std::string> planToAny(const Eigen::VectorXd& q_start, const std::vector<Eigen::Matrix4d>& tforms);
    [[nodiscard]] tl::expected<std::vector<Eigen::VectorXd>, std::string> planLinear(const Eigen::VectorXd& q_start, const Eigen::Matrix4d& tform, int steps = 10, double max_joint_jump = 1.0);
    void addBox(const std::string& name, const Eigen::Vector3d& size, const Eigen::Matrix4d& tform_in_base, const Eigen::Vector4d& rgba = {0.6, 0.6, 0.6, 1.0});
    [[nodiscard]] bool hasBox(const std::string& name) const;
    void remove(const std::string& name);
    void attach(const std::string& name, const Eigen::VectorXd& q);   // re-parent to joint6_flange at q; disable vs gripper_base/left/right
    void detach(const std::string& name, const Eigen::VectorXd& q);   // leave in base_frame where it is at q
    [[nodiscard]] const roboplan::Scene& scene() const;               // for the Rerun robot logger's private-Data FK
    ```
    Construction: `rewriteUrdfForPlanning` → `loadUrdfSceneDescriptionFromXml(xml, meshPackagePaths())`
    → `Scene("emma", ...)` → `addGroupFromChain("arm", "g_base", tip_frame)` →
    `allowAdjacentLinkCollisions()` → `setCollisions(pairs, false)` for `{gripper_left,
    gripper_right, wrist_camera_link} × {joint6, joint6_flange}` restricted to links present in the
    URDF → seed. `q_full_ = getCurrentJointPositions()`, `arm_idx_ = getJointPositionIndices(kArmJoints)`.
    Two `SimpleIk` (`max_iters 200, max_time 0.05, check_collisions`; `max_restarts` 5 and 0, the
    local one keeps linear moves on the seed's IK branch), `RRT` (`rrt_connect, max_connection_distance
    1.0, collision_check_step_size 0.02, max_planning_time`), `PathShortcutter` (`max_step_size 0.02,
    max_iters 200, seed -1 or the seed`). Behaviour: `planJoint` clamps the start, returns an error if
    the goal collides, passes `rrt_.plan` errors through, shortcuts; `planToAny` tries each candidate
    (IK seeded at start, then `planJoint`) and returns the first success with its index, else "no
    candidate reachable"; `planLinear` clamps, local IK, rejects any joint jump > `max_joint_jump`
    and any colliding lerp step; `addBox` replaces an existing name and disables box–box pairs
    (objects resting on each other). `addBoxGeometry/removeGeometry/setCollisions` return
    `tl::expected<void,string>`; throw on error (programming errors).
- `move_arm.cpp`: flag parsing over `rclcpp::remove_ros_arguments(argc, argv)`: `--home | --joints
  q×6 | --pose x y z [r p y] | --linear x y z [r p y]`, `--no-table`, `--v-max`; subscriptions
  `/robot_description` (`rclcpp::QoS(1).transient_local()`) and `/joint_states`;
  `rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>` on
  `/arm_controller/follow_joint_trajectory` with `spin_until_future_complete`; prints the reached TCP
  error via `roboplan::poseError`. Exit 0 on success, 1 otherwise. Same usage text as the Python.
- Tests (`ament_add_gtest`): `test_grasp.cpp` (old `test_grasp.py`: orthonormal rotation / det +1 at
  several yaws, tilt leans toward the reach, `offset` moves along -z by `dist`); `test_planner.cpp`
  (fixture `SetUpTestSuite`, seed 7, table + block boxes; the nine `test_planner.py` checks: home
  collision-free, tcp 45 mm past `gripper_base`, `armPositions` ordering, IK round trip via
  `poseError` (< 1 mm, < 0.5°), block reachable from home (`planToAny` over pregrasps; path starts at
  home, ends at the chosen pregrasp, no collisions), linear grasp + attach + lift (lift rises >
  0.8·0.05·cos 35°), trajectory timing `[0.5, 0.6, 1.1125]` for `[H, H+0.4, H+0.41, H]`, look pose
  (IK for the optical frame 0.2 m from the block, 65° down; `kLookQ` collision-free, reachable,
  block/place/offsets project inside the 640×400 fovy-65° image with margins, block on the optical
  axis), clamp past a joint limit then plan).
  URDF at build time, in `CMakeLists.txt` under `BUILD_TESTING`:
  ```cmake
  find_package(ament_cmake_gtest REQUIRED)
  find_package(xacro REQUIRED)
  find_package(emma_description REQUIRED)
  xacro_add_xacro_file(${emma_description_DIR}/../urdf/emma.urdf.xacro ${CMAKE_CURRENT_BINARY_DIR}/test/emma.urdf)
  add_custom_target(${PROJECT_NAME}_test_urdf DEPENDS ${XACRO_OUTPUT_FILE})
  ament_add_gtest(test_planner test/test_planner.cpp)
  add_dependencies(test_planner ${PROJECT_NAME}_test_urdf)
  target_link_libraries(test_planner ${PROJECT_NAME})
  target_compile_definitions(test_planner PRIVATE EMMA_TEST_URDF="${XACRO_OUTPUT_FILE}")
  ```
  The explicit output path matters: the helper's default output strips `.xacro` next to the input,
  i.e. inside `install/emma_description`. `xacro --deps` runs at configure time and needs `$(find …)`
  to resolve, so the `exec_depend`s on the three description packages (which make colcon build them
  first) are required. Fallback if it misbehaves: `popen("xacro …")` in the test fixture.
- `package.xml`: `<buildtool_depend>ament_cmake`; `<depend>` roboplan, eigen, tinyxml2, sensor_msgs,
  trajectory_msgs, builtin_interfaces, ament_index_cpp, rclcpp, rclcpp_action, control_msgs,
  std_msgs; `<exec_depend>` emma_description, mycobot_description, myagv_description;
  `<test_depend>` ament_cmake_gtest, xacro, ament_lint_auto, ament_cmake_xmllint,
  ament_cmake_lint_cmake. **Not** `ament_lint_common` (it pulls `ament_cmake_uncrustify`, whose ROS
  style conflicts with the Google style required by the user's C++ rules). Keep the sibling
  packages' `set(ament_cmake_copyright_FOUND TRUE)` / `set(ament_cmake_cpplint_FOUND TRUE)` lines
  and `ament_lint_auto_find_test_dependencies()`.
- C++ style: Google baseline + the user's `~/.claude/rules/cpp-style.md` (C++20, `[[nodiscard]]`,
  spelled-out Eigen types, `const` everywhere, `tl::expected<T, std::string>` for recoverable errors,
  `.hpp/.cpp`, `std::string_view` params, init-statements in `if`, no `std::bind`, comments say why,
  full sentences end with a period, no ticket IDs).
- pixi: `test` task becomes `colcon test --packages-select emma_manipulation emma_perception &&
  colcon test-result --verbose`. README `src/emma_manipulation/README.md`: contents table, C++ API
  table (`tl::expected` errors instead of `None`), collision-model notes (tinyxml2 rewrite, mesh
  scales, disabled pairs, attach link), `move_arm` usage; root README package list.
Verify: `pixi run build && pixi run test` (reachability gate and look pose must pass; same seed, same
numbers as the Python tests); `pixi run sim headless:=true rviz:=false` + `pixi run bash -c "source
install/setup.bash && ros2 run emma_manipulation move_arm --home"` and `--pose 0.25 0 0.20 0 3.1416 0`.

### M4 `beet/m4-behaviors` — beet tree, Rerun, demo tasks
`git checkout -b beet/m4-behaviors beet/m3-manipulation`.
- beet: `git submodule add https://github.com/EzraBrooks/beet external_packages/beet`, then
  `git -C external_packages/beet checkout 436617cd2fd3abc5811223b63977b2ec74800c27`; add
  `shallow = true` to the new `.gitmodules` entry (matches the two existing entries). Consumed only
  from `src/emma_behaviors/CMakeLists.txt`:
  ```cmake
  set(BEET_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(BEET_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../external_packages/beet ${CMAKE_CURRENT_BINARY_DIR}/beet EXCLUDE_FROM_ALL)
  ```
  beet has no `package.xml`, so the colcon `--base-paths` in `pixi.toml` stay unchanged. Fallback if
  its CMake misbehaves: a 3-line `add_library(beet INTERFACE)` over `external_packages/beet/include`
  linked to `roboplan::tl_expected` + `Threads::Threads`.
- pixi: `pixi add "librerun-sdk>=0.38.1,<0.39" "rerun-sdk>=0.38.1,<0.39"` (check it resolves next to
  the robostack packages; `librerun-sdk` pulls `arrow-cpp`). Tasks (same as the old branch):
  ```toml
  # Pick and place of the block the wrist camera finds, in the sim (MuJoCo viewer + RViz + Rerun). Args pass through.
  pick-demo = { cmd = "bash -c 'source install/setup.bash && ros2 launch emma_behaviors pick_and_place.launch.py \"$@\"' --", depends-on = ["build"] }
  # Headless pick and place; exits 0 only if the behaviour tree succeeds within 240 s.
  pick-check = { cmd = "bash src/emma_behaviors/scripts/pick_check.sh", depends-on = ["build"] }
  ```
  and `test` adds `emma_behaviors`.
- `src/emma_behaviors` (ament_cmake):
  ```
  CMakeLists.txt package.xml LICENSE README.md
  launch/pick_and_place.launch.py   scripts/pick_check.sh      (git show pick-place/m4-docs:src/emma_behaviors/... ; one-line change below)
  include/emma_behaviors/{ros_io,leaves,tree,viz,rerun_tree}.hpp
  src/{ros_io,leaves,viz,pick_and_place}.cpp
  test/test_tree.cpp
  ```
  Executable `pick_and_place`: `ros_io.cpp leaves.cpp viz.cpp pick_and_place.cpp`; links
  `emma_manipulation::emma_manipulation beet::beet rerun_sdk` (`find_package(rerun_sdk REQUIRED)`,
  target name `rerun_sdk` as in beet's examples) + `ament_target_dependencies(rclcpp rclcpp_action
  control_msgs sensor_msgs geometry_msgs std_msgs ament_index_cpp)`. Install to
  `lib/${PROJECT_NAME}` (so `ros2 run emma_behaviors pick_and_place` and the launch file work
  unchanged); `install(DIRECTORY launch scripts DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)`.
- `package.xml`: depend rclcpp, rclcpp_action, control_msgs, sensor_msgs, geometry_msgs, std_msgs,
  emma_manipulation, ament_index_cpp, eigen; exec_depend launch, launch_ros, emma_simulation,
  emma_perception; test_depend ament_cmake_gtest, xacro, emma_description, ament_lint_auto,
  ament_cmake_xmllint, ament_cmake_lint_cmake, ament_cmake_flake8 (launch file). Rerun has no ROS
  package name; it is a pixi dep only (note in README).
- **ROS I/O** (`ros_io.hpp`): plain struct so tests can fill it without ROS:
  ```cpp
  struct Io {
    rclcpp::Node::SharedPtr node;
    std::optional<sensor_msgs::msg::JointState> joint_state;            // latest with all arm joints
    std::optional<geometry_msgs::msg::PoseStamped> block_pose; std::uint64_t block_pose_count = 0;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm;
    rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SharedPtr gripper;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  };
  [[nodiscard]] tl::expected<std::shared_ptr<Io>, std::string> openIo(rclcpp::Node::SharedPtr node);  // subs + clients + wait_for_action_server(10 s)
  [[nodiscard]] tl::expected<std::string, std::string> waitForUrdf(rclcpp::Node& node, std::chrono::seconds wall_timeout);  // latched /robot_description; wall time because sim time stands still until /clock
  ```
  Topics/actions: `/joint_states` (sensor-data QoS), `/block_pose` (`PoseStamped`, depth 1),
  `/arm_controller/follow_joint_trajectory`, `/gripper_action_controller/gripper_cmd`
  (`ParallelGripperCommand`, `command.name = ["gripper_controller"]`, `position = [0.0 | -0.007]`).
  All callbacks run on the main thread inside `rclcpp::spin_some(node)` called once per tick before
  `runner.tick()`, so `Io` needs no locks. This replaces py_trees' `joints2bb` parallel branch (a
  never-finishing parallel branch is a listed beet gap). Deviation documented in the README.
- **Edge types** (`leaves.hpp`):
  ```cpp
  struct NoPlan { std::string why; };  struct ActionFailed { std::string why; };
  struct NotPlaced { double error_m; };  struct GaveUp {};                 // + beet::Timeout from timeout_ticks
  struct Cell { std::shared_ptr<Io> io; std::shared_ptr<emma_manipulation::ArmPlanner> planner; std::shared_ptr<Viz> viz; };
  struct Looked { Cell cell; geometry_msgs::msg::PoseStamped block; };
  struct Grasps { Cell cell; std::vector<Eigen::Matrix4d> grasp, pregrasp, place, preplace; };
  struct Chosen : Grasps { std::size_t index; };
  template <class S> struct Planned { S state; control_msgs::action::FollowJointTrajectory::Goal goal; };
  ```
  Blackboard mapping: `joint_state` → read from `io` at plan time (every move still starts from the
  latest sample, sag included); `block_pose` → `Looked::block`; the four candidate lists → `Grasps`;
  `grasp_index` → the `Grasps`→`Chosen` type transition (a linear move before a grasp is chosen is a
  compile error, not a runtime FAILURE); `trajectory` → `Planned<S>::goal`. Distinct error structs
  because beet merges same-type errors.
- **Leaves** (`leaves.hpp/.cpp`):
  ```cpp
  Task<Result<Cell, never>>   waitForJoints(Cell);                       // co_await running until io->joint_state
  template <class S> Result<Planned<S>, NoPlan> planJoint(S, const Eigen::VectorXd& q_goal);   // via lambdas for kLookQ / kHomeQ
  Result<Planned<Chosen>, NoPlan> planPregrasp(Grasps);                  // planToAny(pregrasp) -> Chosen{g, index}
  Result<Planned<Chosen>, NoPlan> planPreplace(Chosen);                  // planToAny({preplace[index]})
  enum class Target { Grasp, Pregrasp, Place, Preplace };
  Result<Planned<Chosen>, NoPlan> planLinear(Chosen, Target);            // lambdas: grasp, lift(Pregrasp), place, retreat(Preplace)
  template <class S> Task<Result<S, ActionFailed>> execute(Planned<S>);  // logs EE path to Rerun, async_send_goal, poll futures with co_await running; rejected/aborted/canceled -> ActionFailed
  template <class S> Task<Result<S, ActionFailed>> gripper(S, double position);   // lambdas openGripper<S>, closeGripper<S>
  template <class S> Task<Result<S, never>>  settle(S);                  // kSettleTicks = 5
  template <class S> Task<Result<Looked, never>> waitBlockPose(S);       // snapshot io->block_pose_count on first resume, loop until it grows
  Grasps  computeGrasps(Looked);     // graspCandidates + offsets; places = grasps shifted to kPlaceXyz; planner->addBox("block"); viz box; arm_base_xy = fk(kHomeQ, "g_base") (fixed in base_link)
  Chosen  attachBlock(Chosen);  Chosen detachBlock(Chosen);  Cell detachIfHeld(Cell);
  Result<Cell, NotPlaced> checkPlaced(Looked);   // 20 mm vs kPlaceXyz
  Result<Cell, GaveUp>    giveUp(Cell);
  ```
  Function templates are passed as `&execute<Chosen>` (beet's `callable_traits` handles function
  pointers); parameterised leaves are non-generic lambdas (`[](Cell c) { return planJoint(std::move(c),
  kLookQ); }`), which `beet::offload`'s `callable_traits` requires. Pose helpers: `poseToTform
  (PoseStamped) -> Matrix4d`, `yawOf(PoseStamped)`.
- **Tree** (`tree.hpp`, a header because the type is the tree):
  ```cpp
  template <class S> auto look() {   // S = Cell (first look) and S = Chosen (check look)
    return beet::sequence(
        beet::named<"plan look">(beet::offload([](S s) { return planJoint(std::move(s), kLookQ); })),
        beet::named<"execute look">(&execute<S>),
        beet::named<"settle">(&settle<S>),
        beet::named<"detect">(beet::timeout_ticks(kDetectTicks /*50*/, &waitBlockPose<S>)));
  }
  inline auto makeTree(Cell cell) {
    auto pick_place = beet::sequence(look<Cell>(),
        beet::named<"grasp poses">(computeGrasps), beet::named<"open gripper">(openGripper<Grasps>),
        beet::named<"plan pregrasp">(beet::offload(planPregrasp)), beet::named<"execute pregrasp">(&execute<Chosen>),
        beet::named<"plan grasp">(planGrasp), beet::named<"execute grasp">(&execute<Chosen>),
        beet::named<"close gripper">(closeGripper<Chosen>), beet::named<"attach block">(attachBlock),
        beet::named<"plan lift">(planLift), beet::named<"execute lift">(&execute<Chosen>),
        beet::named<"plan preplace">(beet::offload(planPreplace)), beet::named<"execute preplace">(&execute<Chosen>),
        beet::named<"plan place">(planPlace), beet::named<"execute place">(&execute<Chosen>),
        beet::named<"release">(openGripper<Chosen>), beet::named<"detach block">(detachBlock),
        beet::named<"plan retreat">(planRetreat), beet::named<"execute retreat">(&execute<Chosen>),
        look<Chosen>(), beet::named<"check placed">(checkPlaced),
        beet::named<"plan home">(beet::offload(planHome<Cell>)), beet::named<"execute home">(&execute<Cell>));
    auto recover = beet::sequence(
        beet::recover<ActionFailed, NoPlan>(
            beet::sequence(beet::named<"recover: open">(openGripper<Cell>), beet::named<"recover: detach">(detachIfHeld),
                           beet::named<"recover: plan home">(beet::offload(planHome<Cell>)), beet::named<"recover: execute home">(&execute<Cell>)),
            [cell](const auto& e) { cell.viz->logText(describe(e)); return cell; }),   // handlers see only the error, so Cell is captured at build time
        beet::named<"give up">(giveUp));
    return beet::sequence(beet::named<"wait for joints">(waitForJoints),
                          beet::fallback(beet::named<"retry">(beet::retry(2, beet::named<"pick and place">(pick_place))),
                                         beet::named<"recover">(recover)));
  }
  using Tree = decltype(makeTree(std::declval<Cell>()));
  static_assert(std::is_same_v<beet::output_t<Tree>, Cell>);
  static_assert(std::is_same_v<beet::error_t<Tree>, GaveUp>);   // the only failure left is the deliberate one
  ```
  Semantics mapping:
  - Selector/Retry(2) → `fallback(retry(2, …), recover)`: beet `retry(2)` = 2 attempts total =
    py_trees `Retry(num_failures=2)`; `fallback`'s error is the last child's, so `pick_place`'s
    `variant<NoPlan, ActionFailed, beet::Timeout, NotPlaced>` never reaches the root. The recover
    branch handles its own `ActionFailed`/`NoPlan` so the root's only error is `GaveUp`. Assert
    `error_t<Tree> == GaveUp` rather than `never`: wrapping the root in `recover<GaveUp>` would make
    the root always succeed, which turns the StatusTable root green and forces the exit code to come
    from a value instead of `Status`.
  - Fresh `/block_pose` + 10 s → `timeout_ticks(50, waitBlockPose)` at 200 ms ticks. Timeout destroys
    the inner coroutine; `waitBlockPose` holds no resources.
  - Settle 1 s → 5 ticks. No wall timers in beet; tick counts are deterministic.
  - RRT in `beet::offload` (look, pregrasp, preplace, home); linear plans and `computeGrasps` run
    in-tick (milliseconds).
  - Thread safety: `ArmPlanner` stays non-thread-safe. It is touched by (a) the offload thread while
    the sequence is parked on that node and (b) main-thread leaves at other times, never both,
    because every planner leaf is inside one `sequence`. The main loop's Rerun robot logger must not
    use `ArmPlanner::fk`; it uses `scene().forwardKinematics(private_data, q, …)`, which only reads
    the immutable model (documented). ROS callbacks touch only `Io`.
  - Exceptions thrown inside leaves are rethrown by `Runner::tick()`; treat as fatal (terminate →
    nonzero), no blanket catch.
  - beet awkwardness to know: `sequence` keeps every intermediate output alive until it finishes
    (fine, `Planned` is a few KB); `retry`/`fallback` copy the input per attempt (`Cell` is three
    `shared_ptr`s); a halted `offload` job runs to completion detached; `execute` destroyed mid-goal
    does not cancel the action goal (no halt path reaches it in this tree; add an `async_cancel_goal`
    guard if one appears).
- **Main loop** (`pick_and_place.cpp`): `rclcpp::init` → node `pick_and_place` → `waitForUrdf(30 s)`
  → `ArmPlanner` + table box (`addBox("table", 2*half, translation(center), {0.55,0.4,0.25,1})`) →
  `openIo` → `Viz` (`--save file.rrd` else `spawn()`) → `logUrdf`, table box → `GraphLogger<Tree>`,
  `StatusTable<Tree>`, `Runner{tree, cell, table}`; 200 ms loop: `rec.set_time_sequence("tick", n)`,
  `set_time_duration_secs("time", wall)`, `rclcpp::spin_some`, `robot.log(joint_state)`,
  `status = runner.tick()`, `graph.log(table)`, `sleep_until(next)` while `status == Running &&
  rclcpp::ok()`; afterwards print one line per node from `beet::tree_info<Tree>` + `table.status(i)`
  indented by depth (the py_trees `unicode_tree` replacement for `pick_check` logs); exit
  `status == Success ? 0 : 1`.
- **Rerun** (`viz.hpp/.cpp`, pimpl so only `viz.cpp`, `pick_and_place.cpp` and the graph logger
  include `rerun.hpp`): `explicit Viz(std::optional<std::filesystem::path> save)` (`save(path)` else
  `spawn()`; a failed spawn logs a warning and continues disconnected), `static Viz disabled()`
  (`RecordingStream::disabled()`, for tests), `logUrdf(xml, package_paths)`, `logBox(name, size,
  tform, color)` (`Boxes3D::from_centers_and_half_sizes` with `rerun::CoordinateFrame("base_link")`,
  the URDF link frame planning uses, not `"world"` as in the UR5 example), `logPath(name, points)`
  (`LineStrips3D` under `base_link/plan/<name>`), `logText(text, level)` (`TextLog` at `log`),
  `setTick(tick, seconds)`, `stream()`.
  - URDF: the planning-rewritten XML is written to
    `std::filesystem::temp_directory_path() / "emma_pick_and_place.urdf"` (the `.urdf` extension
    selects Rerun's loader), `setenv("ROS_PACKAGE_PATH", <package_paths joined by ':'>, 1)`, then
    `log_file_from_path(path, "world/robot", true)`. The rewritten URDF is logged because it carries
    the mesh `scale` fixes on the visual tags too; if the viewer honours COLLADA `<unit>` and shows
    the gripper/base double-scaled, switch to the raw `/robot_description` string (one line). Verify
    in the viewer on the first `pick-demo`.
  - `RobotLogger` (in `viz.cpp`, modelled on the example's `Robot::log`): built from
    `planner->scene()`, owns a `pinocchio::Data`; per tick builds the full q from the `JointState`
    names present in the model (`getJointPositionIndices`), and logs
    `Transform3D::from_translation_mat3x3(...).with_parent_frame(parent).with_child_frame(child)` at
    `world/robot/joints/<joint>` for each BODY frame whose parent frame is a JOINT. Right finger
    stays at 0 (mimic dropped), acceptable.
  - Boxes: table once (static) at startup; block re-logged by `computeGrasps` (detected pose) and
    `detachBlock` (where released); not animated while held.
  - Paths: `execute<S>` logs the EE path of `Planned::goal` (FK of each point, main thread) under
    `base_link/plan/<label>` before sending the goal; text events for plan results, action results,
    grasp index, check error, recover.
  - Tree graph: copy `external_packages/beet/examples/rerun_tree.hpp` to
    `include/emma_behaviors/rerun_tree.hpp` with an MIT attribution comment and one change:
    `shorten()` also strips `emma_behaviors::` and `emma_manipulation::` so labels stay narrow.
  - `--save <file.rrd>` parsed from `rclcpp::remove_ros_arguments(argc, argv)`.
    `scripts/pick_check.sh`: add `--save log/pick_check.rrd` before `--ros-args` so headless runs
    never spawn a viewer. Launch file unchanged (demo spawns the viewer).
- `test_tree.cpp` (mirrors the old `test_behaviours.py`; same xacro CMake lines as M3; no ROS comms;
  `Cell{std::make_shared<Io>(), planner, std::make_shared<Viz>(Viz::disabled())}` with
  `io->joint_state` set by hand; call `planPregrasp` directly, not through `offload`, so no threads):
  the `static_assert`s in `tree.hpp`; `beet::tree_info<Tree>` contains the labels `wait for joints,
  retry, recover, plan look, detect, close gripper, check placed, execute home`; `computeGrasps`
  sizes/translations/offsets (grasps at the block, places at `kPlaceXyz` with the grasp rotation,
  preplace `kPreOffset` from place) and `hasBox("block")`; `planPregrasp` yields an index and a goal
  in `kArmJoints` order, then `planLinear(Grasp)` from the pregrasp end reaches the chosen grasp
  within 1 mm; `checkPlaced` near (1 cm off → success) / far (block start → `NotPlaced`);
  `detachIfHeld` without a block succeeds.
- Docs:
  - `src/emma_behaviors/README.md`: beet intro (typed edges, no blackboard); contents table; tree
    diagram annotated with the edge types (`Cell → Looked → Grasps → Chosen → … → Cell`); "Error
    types" table replacing "Blackboard keys"; tick/timeout semantics (5 Hz, 5 settle ticks, 50 detect
    ticks); Rerun entities table (`tree`, `world/robot`, `base_link/table`, `base_link/block`,
    `base_link/plan/*`, `log`) and `--save`; failure handling (retry/recover/GaveUp, exit codes);
    deviations from the py_trees version (no parallel root, linear-before-choose is a compile error);
    pointer to `docs/beet-feedback.md`. The old `git show pick-place/m4-docs:src/emma_behaviors/README.md`
    has the running instructions, launch-argument table and caveats to carry over.
  - Root `README.md`: package list (emma_manipulation "C++ library + `move_arm`", emma_behaviors
    "beet behaviour tree … logged to Rerun"), `pick-demo` note that the Rerun viewer opens,
    `pick-check` writes `log/pick_check.rrd` (`pixi run rerun log/pick_check.rrd` to replay),
    `pixi run test` runs gtest + pytest; link to `docs/beet-feedback.md`.
  - `AGENTS.md` "External packages": `external_packages/beet` is a header-only C++20 library pinned at
    `436617cd`, not a colcon package, consumed by `add_subdirectory` from `src/emma_behaviors`, must
    not be edited; pick-and-place runs need the Rerun viewer (`rerun-sdk` in pixi).
Verify: `pixi run build && pixi run test`; `pixi run pick-check` → `pick-check: PASS` and
`log/pick_check.rrd` written; `pixi run pick-demo` → viewer shows URDF at correct scale, graph
colours change per tick, EE paths, block box moves to `kPlaceXyz`.

### beet feedback doc (grows through M4, committed with it)
`docs/beet-feedback.md`: issues, shortcomings and bugs hit while using beet, written so it can be
handed to the colleague as-is. One entry per item: what we tried, what happened (exact compiler /
runtime text), workaround used, suggested fix, beet commit. Seed with what is already known:
- No way to run a never-finishing side branch (joint-state cache) under `parallel_*`: children must
  share one input type and `parallel_any` reports `variant<Out, never>` instead of `Out` (README
  gap). Worked around with a subscription cache outside the tree.
- `recover` handlers see only the error, not the input, so recovery captures `Cell` at build time.
- Only tick-count timeouts (`timeout_ticks`), no wall-clock timeout/timer decorator.
- Resources must ride through every intermediate output (`Cell` in every state struct).
- Halting an `offload` job only requests a stop; the thread runs to completion, detached.
- `rerun_tree.hpp` (tree → Rerun graph) lives in `examples/`, not the library, so consumers copy it.
- No `install()` / `beetConfig.cmake` export in `CMakeLists.txt`: only consumable by
  `add_subdirectory`/FetchContent, not `find_package(beet)`; not on conda-forge.
- Parameterised leaves must be non-generic (`callable_traits`), so `offload(planJoint<S>)` needs a
  lambda wrapper per use.
- `sequence` keeps every intermediate output alive until it finishes; `retry`/`fallback` copy the
  input per attempt.
Plus anything new found during the port (compile-error quality, GCC 15 issues, tracing limits with
`AnyNode` / coroutine leaves, etc.). Mention the doc from the root README and `emma_behaviors` README.

### Housekeeping
- PRs #2–#5 are closed (done). Do not delete the `pick-place/*` branches.
- Each milestone: push the branch, open a PR against the previous milestone branch (M1 against
  `main`), as the old stack did.

## Risks
- **beet pre-alpha / AI-generated**: pin the commit; copy `rerun_tree.hpp` rather than include from
  `examples/`; keep all beet use in `tree.hpp`/`leaves.hpp` so an API change is one file.
- **GCC 15 + coroutines**: expected fine (beet CI uses conda `cxx-compiler`); keep default flags, no
  `-march=native` (coroutine frames are allocated with plain `operator new`, 16-byte alignment; AVX
  would make `Eigen::Matrix4d` members require 32).
- **Rerun C++ SDK**: heavy headers, ~30 s per TU (isolated to `viz.cpp` + main); `librerun-sdk`
  pulls `arrow-cpp`, check `pixi add` resolves with robostack; URDF loader's COLLADA unit handling is
  unverified (fallback noted above).
- **roboplan C++ vs Python**: `RRT::plan` returns `tl::expected`, no exception path; other `Scene`
  calls throw on bad frame names only (fatal by design). `addBoxGeometry/removeGeometry/setCollisions`
  return `tl::expected<void, string>` and must be checked.
- **xacro at build time**: needs `xacro` on PATH during `colcon build` (true in pixi) and the
  description packages built first (package.xml deps). Fallback: `popen("xacro …")` in the fixture.
- **Lint**: `ament_lint_common` would run uncrustify against Google-style code; keep it out of both
  C++ packages' test_depends.
- **Detached offload threads** keep running if the process exits mid-plan (not on the normal paths).

## Verification (end to end)
1. `pixi run build` clean on each milestone branch.
2. `pixi run test`: gtest for `emma_manipulation` (reachability gate, look pose) and
   `emma_behaviors` (tree builds, typed-edge checks), pytest for `emma_perception`.
3. `pixi run pick-check` passes headless within 240 s; `.rrd` written.
4. `pixi run pick-demo` visual check in Rerun + RViz; `move_arm --home` works.
5. READMEs audited per milestone (root, description, simulation, perception, manipulation,
   behaviors) and `AGENTS.md` updated.
