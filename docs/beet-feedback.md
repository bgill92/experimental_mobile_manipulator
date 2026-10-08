# beet feedback from emma's pick and place

Issues, shortcomings and bugs we hit while porting emma's py_trees pick and place
(`src/emma_behaviors`) to [beet](https://github.com/EzraBrooks/beet). Written to be handed to
beet's author as is.

All entries are against beet `436617cd2fd3abc5811223b63977b2ec74800c27` (2026-10-07), built with
conda-forge GCC 15.3.0 (`x86_64-conda-linux-gnu-g++`), `-std=c++20`, tl-expected 1.3.1 from
conda-forge. Overall the port went smoothly: the whole tree, its `static_assert`s on the root's
output and error types, and the traced `Runner` all compiled first time on GCC 15 with
`-Wall -Wextra -Wpedantic` and no warnings from beet's headers.

Each entry: **Tried**, **Happened**, **Workaround**, **Suggestion**.

## 1. No way to run a never-finishing side branch under `parallel_*`

- **Tried:** py_trees ran a `joints2bb` subscriber as a parallel branch next to the mission, so
  every behaviour saw the latest `/joint_states`. The beet equivalent is a coroutine
  `Task<Result<never, never>>(Cell)` that loops on `co_await beet::running`, next to the mission
  under `parallel_any`.
- **Happened:** `parallel_any` reports `variant<Out, never>` instead of `Out` (listed in beet's
  README gaps). A probe with a `Cell(Cell)` work leaf and a never-finishing cache leaf gives:
  ```text
  error: conversion from 'int' to non-scalar type 'beet::output_t<beet::Node<beet::detail::parallel_impl<beet::detail::any_policy, beet::Node<beet::detail::lift_impl<Cell (*)(Cell), Cell>, Cell, Cell, beet::never>, beet::Node<beet::detail::lift_impl<beet::Task<tl::expected<beet::never, beet::never> > (*)(Cell), Cell>, Cell, beet::never, beet::never> >, Cell, std::variant<Cell, beet::never>, beet::never> >' {aka 'std::variant<Cell, beet::never>'} requested
  ```
  Parallel children must also share one input type, so the cache branch has to take the whole
  `Cell` even though it needs only the I/O.
- **Workaround:** the subscription cache lives outside the tree (`Io`, filled by ROS callbacks in
  `spin_some` before each `Runner::tick()`); leaves read it through `Cell`.
- **Suggestion:** drop `never` from `parallel_any`'s output union (`union_t` already drops it from
  error sets), so `variant<Out, never>` collapses to `Out`.

## 2. `recover` handlers see only the error, not the input

- **Tried:** recovery that logs and returns the `Cell` the failed branch started with.
- **Happened:** the handler is called with the error only (README gap).
- **Workaround:** `makeTree(Cell cell)` captures `cell` in the handler lambda at build time.
- **Suggestion:** also accept handlers callable as `handler(input, error)`.

## 3. Only tick-count timeouts

- **Tried:** py_trees waited 10 s for a fresh `/block_pose` and settled 1 s at the look pose.
- **Happened:** beet has `timeout_ticks(n, node)` only; no wall-clock timeout or timer decorator.
- **Workaround:** a fixed 200 ms tick, so 50 ticks for the detection timeout and 5 settle ticks.
  Deterministic, but tied to the tick rate and stretched by any slow tick.
- **Suggestion:** a `timeout(duration, node)` / `wait(duration)` pair on a pluggable clock (so a
  ROS sim clock could drive it).

## 4. Resources ride through every intermediate output

- **Tried:** leaves need the I/O, the planner and the Rerun logger.
- **Happened:** a resource reaches later nodes only through node outputs (README gap), so every
  state struct (`Looked`, `Grasps`, `Chosen`, `Planned<S>`) carries a `Cell` it mostly passes
  through, and generic leaves need a `cellOf(state)` helper to find it.
- **Workaround:** `Cell` is three `shared_ptr`s, embedded in every state.
- **Suggestion:** a context/environment parameter the `Runner` hands to every leaf (typed, like
  the input), separate from the data flowing along edges.

## 5. Halting an `offload` job only requests a stop

- **Tried:** RRT plans in `beet::offload`.
- **Happened:** a halted job keeps running on its detached thread until it returns; its result is
  dropped. roboplan's RRT takes no `std::stop_token`, so a halt (e.g. a timeout around a plan)
  would leave the planner busy on another thread while the tree moves on.
- **Workaround:** no halt path reaches an offloaded plan in emma's tree, and every planner leaf is
  in one `sequence`, so the non-thread-safe planner is never used by two threads at once.
- **Suggestion:** document this prominently next to `offload`, or offer a variant that keeps the
  node `Running` (or joins) until the job has really finished after a halt.

## 6. `rerun_tree.hpp` lives in `examples/`

- **Tried:** the tree status graph in Rerun.
- **Happened:** the graph logger is an example, not part of the library.
- **Workaround:** copied to `src/emma_behaviors/include/emma_behaviors/rerun_tree.hpp` with an
  attribution comment, plus one change: `shorten()` also strips our namespaces.
- **Suggestion:** ship it as an optional header (e.g. `beet/rerun.hpp`, only usable when
  `rerun_sdk` is found), with a configurable list of prefixes to strip.

## 7. No `install()` or package config

- **Tried:** consuming beet from a colcon workspace.
- **Happened:** `CMakeLists.txt` has no `install()` / `beetConfig.cmake` export, so beet works with
  `add_subdirectory` or FetchContent only, not `find_package(beet)`; it is not on conda-forge.
  (Its own `FetchContent(tl-expected FIND_PACKAGE_ARGS 1.1)` resolved offline against the
  installed tl-expected 1.3.1, which was nice.)
- **Workaround:** a git submodule at `external_packages/beet`, pulled in with
  `add_subdirectory(... EXCLUDE_FROM_ALL)` and `BEET_BUILD_TESTS` / `BEET_BUILD_EXAMPLES` forced
  off.
- **Suggestion:** install the headers and export `beet::beet` with a config file; a conda-forge
  recipe would then be small.

## 8. Parameterised leaves must be non-generic

- **Tried:** `beet::offload` around a plan whose state type is a template parameter.
- **Happened:** `offload` reads the argument type through `callable_traits`, so generic lambdas
  (and overload sets) do not work. A probe `beet::offload([](auto c) { return c; })` gives:
  ```text
  external_packages/beet/include/beet/meta.hpp:155:8: error: 'decltype' cannot resolve address of overloaded function
  external_packages/beet/include/beet/offload.hpp:61:9: error: no type named 'args' in 'struct beet::detail::callable_traits<<lambda(auto:42)> >'
  ```
  The error points into beet's internals rather than saying the callable must be non-generic.
- **Workaround:** one function template per parameterised plan (`planLook<S>`, `planHome<S>`),
  passed as `&planLook<Cell>`.
- **Suggestion:** an `offload<In>(fn)` overload like `node<In>(fn)`, and a `static_assert` with a
  readable message when `callable_traits` cannot see the arguments.

## 9. Intermediate outputs and per-attempt copies

- **Tried:** a 23-step pick-and-place `sequence` under `retry(2, ...)` and `fallback`.
- **Happened:** `sequence` keeps every intermediate output alive until it finishes (one
  `std::optional` per child in its frame), and `retry` / `fallback` copy the input per attempt.
- **Workaround:** none needed: the outputs are a few KB (`Planned` holds a trajectory) and `Cell`
  is cheap to copy.
- **Suggestion:** destroy each intermediate value once the next child has consumed it.

## 10. Offloaded functions share state the type system cannot see

- **Tried:** planning leaves read the latest joint state, which ROS callbacks on the main thread
  update.
- **Happened:** the planning leaves run on `offload` threads while the main thread keeps spinning
  callbacks, so reading the joint-state cache from them is a data race. Nothing in beet flags this
  (README gap: offloaded functions' thread safety is unchecked); it only shows up on review.
- **Workaround:** a mutex around the joint state in `Io` (`armQ()` / `storeJointState()`).
- **Suggestion:** a way to snapshot inputs on the tick thread before the job starts (e.g.
  `offload(prepare, job)`, where `prepare` runs in the tick and `job` gets its result), so jobs
  only see values they own.
