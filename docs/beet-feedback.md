# beet feedback from emma's pick and place

Issues, shortcomings and bugs we hit while porting emma's py_trees pick and place
(`src/emma_behaviors`) to [beet](https://github.com/EzraBrooks/beet). Written to be handed to
beet's author as is.

Entries 1 to 10 were found against beet `436617cd2fd3abc5811223b63977b2ec74800c27` (2026-10-07)
and are still open at `090bfa4` (0.2.0, 2026-10-22), the version we now use; entries 11 and 12
came from that bump. Issues fixed upstream are removed from this list. Built with conda-forge
GCC 15.3.0 (`x86_64-conda-linux-gnu-g++`), `-std=c++20`, tl-expected 1.3.1 from conda-forge.
Overall the port went smoothly: the whole tree, its `static_assert`s on the root's
output and error types, and the traced `Runner` all compiled first time on GCC 15 with
`-Wall -Wextra -Wpedantic` and no warnings from beet's headers.

Each entry: **Tried**, **Happened**, **Workaround**, **Suggestion**; entries 1 to 10 also have their
**Status** at `090bfa4`.

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
- **Status at `090bfa4`:** open; still listed under the README's current limitations.

## 2. `recover` handlers see only the error, not the input

- **Tried:** recovery that logs and returns the `Cell` the failed branch started with.
- **Happened:** the handler is called with the error only (README gap).
- **Workaround:** `makeTree(Cell cell)` captures `cell` in the handler lambda at build time.
- **Suggestion:** also accept handlers callable as `handler(input, error)`.
- **Status at `090bfa4`:** open. The README now presents it as a design choice: errors should
  carry the context their handler needs, and anything else is captured when the tree is built.

## 3. Only tick-count timeouts

- **Tried:** py_trees waited 10 s for a fresh `/block_pose` and settled 1 s at the look pose.
- **Happened:** beet has `timeout_ticks(n, node)` only; no wall-clock timeout or timer decorator.
- **Workaround:** a fixed 200 ms tick, so 50 ticks for the detection timeout and 5 settle ticks.
  Deterministic, but tied to the tick rate and stretched by any slow tick.
- **Suggestion:** a `timeout(duration, node)` / `wait(duration)` pair on a pluggable clock (so a
  ROS sim clock could drive it).
- **Status at `090bfa4`:** partly covered. `rate<K>(node)` and `Runner::run(period)` pace the
  tree on a drift-free schedule with overrun handling (`Skip` / `Burst` / `Delay`), so tick counts
  track wall time better than a hand-written sleep loop. There is still no wall-clock timeout, and
  emma still ticks by hand with `Runner::tick()`.

## 4. Resources ride through every intermediate output

- **Tried:** leaves need the I/O, the planner and the Rerun logger.
- **Happened:** a resource reaches later nodes only through node outputs (README gap), so every
  state struct (`Looked`, `Grasps`, `Chosen`, `Planned<S>`) carries a `Cell` it mostly passes
  through, and generic leaves need a `cellOf(state)` helper to find it.
- **Workaround:** `Cell` is three `shared_ptr`s, embedded in every state.
- **Suggestion:** a context/environment parameter the `Runner` hands to every leaf (typed, like
  the input), separate from the data flowing along edges.
- **Status at `090bfa4`:** open. The README's design philosophy now calls it the tradeoff of
  passing data along edges.

## 5. `rerun_tree.hpp` lives in `examples/`

- **Tried:** the tree status graph in Rerun.
- **Happened:** the graph logger is an example, not part of the library.
- **Workaround:** copied to `src/emma_behaviors/include/emma_behaviors/rerun_tree.hpp` with an
  attribution comment, plus one change: `shorten()` also strips our namespaces.
- **Suggestion:** ship it as an optional header (e.g. `beet/rerun.hpp`, only usable when
  `rerun_sdk` is found), with a configurable list of prefixes to strip.
- **Status at `090bfa4`:** open; upstream only modernised the example (`std::format`,
  `using enum`).

## 6. Parameterised leaves must be non-generic

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
- **Status at `090bfa4`:** open; `offload` still reads the argument type through `callable_traits`.

## 7. Intermediate outputs and per-attempt copies

- **Tried:** a 23-step pick-and-place `sequence` under `retry(2, ...)` and `fallback`.
- **Happened:** `sequence` keeps every intermediate output alive until it finishes (one
  `std::optional` per child in its frame), and `retry` / `fallback` copy the input per attempt.
- **Workaround:** none needed: the outputs are a few KB (`Planned` holds a trajectory) and `Cell`
  is cheap to copy.
- **Suggestion:** destroy each intermediate value once the next child has consumed it.
- **Status at `090bfa4`:** open; `sequence` still keeps a tuple of `std::optional`s, one per child.

## 8. Offloaded functions share state the type system cannot see

- **Tried:** planning leaves read the latest joint state, which ROS callbacks on the main thread
  update.
- **Happened:** the planning leaves run on `offload` threads while the main thread keeps spinning
  callbacks, so reading the joint-state cache from them is a data race. Nothing in beet flags this
  (README gap: offloaded functions' thread safety is unchecked); it only shows up on review.
- **Workaround:** a mutex around the joint state in `Io` (`armQ()` / `storeJointState()`).
- **Suggestion:** a way to snapshot inputs on the tick thread before the job starts (e.g.
  `offload(prepare, job)`, where `prepare` runs in the tick and `job` gets its result), so jobs
  only see values they own.
- **Status at `090bfa4`:** open; still listed under the README's current limitations.

## 9. Unlabelled nodes in traces show only their kind

- **Tried:** printing the final tree from `beet::tree_info<Tree>` and `StatusTable` at the end of
  `pick_and_place` (the replacement for py_trees' `unicode_tree` in the `pick-check` log).
- **Happened:** `named<"x">` labels the node it wraps, so in `named<"detect">(timeout_ticks(50,
  &waitBlockPose<S>))` the timeout is "detect" and the leaf under it prints as `leaf`. Unnamed
  composites print as `sequence` / `fallback`, and `recover<...>` prints as `recover`, which in our
  tree sits directly under a sequence we named "recover". `node_info` also has no depth, so every
  consumer recomputes it from `parent`.
- **Workaround:** fall back to `kind` when `label` is empty and compute depth from `parent` (IDs
  are depth-first, so one forward pass); the leaf's name is still lost.
- **Suggestion:** default a leaf's label to its function's name when it is a function pointer, and
  add `depth` to `node_info`.
- **Status at `090bfa4`:** open; `node_info` is unchanged (`kind`, `label`, type names, `parent`).

## 10. No hook to observe an error that a later node swallows

- **Tried:** logging why each pick-and-place attempt failed (`beet::Timeout` from `detect`,
  `NotPlaced` from `check placed`) when `retry(2, ...)` and `fallback` discard those errors.
- **Happened:** beet has no pass-through error hook; `finally` sees neither the result nor the
  error. Timeouts raised by `timeout_ticks` have no leaf of ours to log from.
- **Workaround:** `recover<NoPlan, ActionFailed, NotPlaced, beet::Timeout, beet::Exception>(pick_place, handler)`
  whose generic handler logs and returns the same error, so the error set is unchanged. It adds a
  `recover` node to the graph that never recovers anything.
- **Suggestion:** an `on_error(n, fn)` (or `tap`) decorator that calls `fn(input, error)` and
  passes the result through, with the input available so handlers need not capture it.
- **Status at `090bfa4`:** open. Since 0.2.0 the workaround must also list `beet::Exception`, or a
  throw inside `pick_place` is retried without a log line.

## 11. 0.2.0's exceptions-as-errors change widened every error set silently

- **Tried:** bumping from `436617cd` to `090bfa4`.
- **Happened:** any leaf or handler that is not `noexcept` now has `beet::Exception` in its error
  set. That covered every leaf in emma's tree. The only signal was our own
  `static_assert(std::is_same_v<beet::error_t<Tree>, GaveUp>)`, which failed with just
  `static assertion failed` and did not show the new type. Printing it took an incomplete-type
  trick (`template <class> struct Show; Show<beet::error_t<Tree>> x;`), which gave
  `std::variant<beet::Exception, GaveUp>`. Without that assert, the change would have gone unseen:
  a throw inside `retry` used to escape `Runner::tick()` and crash the node, but it is now an
  ordinary error that `retry` and `fallback` swallow. `retry<N>`, `timeout_ticks<N>` and
  `beet::Listeners` also changed in this release, with no changelog or migration notes.
- **Workaround:** `beet::Exception` added to both `recover` lists, and `noexcept` added to
  `waitForJoints`, `giveUp` and the recovery handler, so the root's error set is `GaveUp` again.
- **Suggestion:** a changelog or release notes for breaking changes. Also a helper that names the
  actual error set when an assert on it fails (e.g. `beet::assert_errors<Tree, GaveUp>()`, whose
  failure instantiates a template with the real type in its name).

## 12. A `Result<T, never>` leaf can still fail

- **Tried:** `Task<Result<Cell, never>> waitForJoints(Cell)`, the tree's first leaf, which sits
  before the `fallback` and so outside every `recover`.
- **Happened:** the signature reads as "cannot fail", but because the coroutine was not `noexcept`
  the node's error set was `beet::Exception`, and that alone put `beet::Exception` on the root. The
  same applies to `recover` handlers: `[cell](const auto& error) { ... }` adds `beet::Exception`
  unless it is `noexcept`.
- **Workaround:** mark `waitForJoints` and the recovery handler `noexcept`.
- **Suggestion:** say next to `never` in the README that a leaf is infallible only when it is
  `noexcept` and returns `Result<T, never>`.
