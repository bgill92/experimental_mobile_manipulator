#pragma once

/// @file
/// The pick-and-place behaviour tree. A header, because the tree's type is the tree: its shape,
/// labels, edge types and error set all live in `Tree`.
///
/// Edge types along the main sequence:
///   Cell -> Looked -> Grasps -> Chosen -> ... -> Chosen -> Looked -> Cell
/// RRT plans run in beet::offload (look, pregrasp, preplace, home); linear plans and the grasp
/// poses take milliseconds and run in the tick. Every planner leaf sits in one sequence, so the
/// non-thread-safe ArmPlanner is never used by two threads at once.

#include <type_traits>
#include <utility>

#include <beet/beet.hpp>

#include "emma_behaviors/leaves.hpp"

namespace emma_behaviors {

/// Move to the look pose, settle, and wait for a fresh detection. S = Cell for the first look and
/// S = Chosen for the check after placing.
template <class S>
[[nodiscard]] auto look() {
  return beet::sequence(
      beet::named<"plan look">(beet::offload(&planLook<S>)),
      beet::named<"execute look">(&execute<S>), beet::named<"settle">(&settle<S>),
      beet::named<"detect">(beet::timeout_ticks(kDetectTicks, &waitBlockPose<S>)));
}

/// @param cell Captured for the recovery handler, which sees only the error.
[[nodiscard]] inline auto makeTree(Cell cell) {
  auto pick_place = beet::sequence(
      look<Cell>(), beet::named<"grasp poses">(&computeGrasps),
      beet::named<"open gripper">(&openGripper<Grasps>),
      beet::named<"plan pregrasp">(beet::offload(&planPregrasp)),
      beet::named<"execute pregrasp">(&execute<Chosen>), beet::named<"plan grasp">(&planGrasp),
      beet::named<"execute grasp">(&execute<Chosen>),
      beet::named<"close gripper">(&closeGripper<Chosen>),
      beet::named<"attach block">(&attachBlock), beet::named<"plan lift">(&planLift),
      beet::named<"execute lift">(&execute<Chosen>),
      beet::named<"plan preplace">(beet::offload(&planPreplace)),
      beet::named<"execute preplace">(&execute<Chosen>), beet::named<"plan place">(&planPlace),
      beet::named<"execute place">(&execute<Chosen>),
      beet::named<"release">(&openGripper<Chosen>), beet::named<"detach block">(&detachBlock),
      beet::named<"plan retreat">(&planRetreat), beet::named<"execute retreat">(&execute<Chosen>),
      look<Chosen>(), beet::named<"check placed">(&checkPlaced),
      beet::named<"plan home">(beet::offload(&planHome<Cell>)),
      beet::named<"execute home">(&execute<Cell>));

  // retry and fallback swallow pick_place's errors, so log each attempt's here. The handler
  // returns the error unchanged; beet has no pass-through error hook.
  auto logged = beet::recover<NoPlan, ActionFailed, NotPlaced, beet::Timeout>(
      std::move(pick_place), [cell]<class E>(E error) -> Result<Cell, E> {
        logEvent(cell, "attempt failed: " + describe(error), Viz::Level::kWarning);
        return beet::make_unexpected(std::move(error));
      });

  // Handles its own ActionFailed / NoPlan, so the only error left is the deliberate GaveUp.
  auto recover = beet::sequence(
      beet::recover<ActionFailed, NoPlan>(
          beet::sequence(beet::named<"recover: open">(&openGripper<Cell>),
                         beet::named<"recover: detach">(&detachIfHeld),
                         beet::named<"recover: plan home">(beet::offload(&planHome<Cell>)),
                         beet::named<"recover: execute home">(&execute<Cell>)),
          [cell](const auto& error) {
            logEvent(cell, "recovery failed: " + describe(error), Viz::Level::kError);
            return cell;
          }),
      beet::named<"give up">(&giveUp));

  // beet's retry(2) is two attempts in total, like py_trees' Retry(num_failures=2). fallback's
  // error is its last child's, so pick_place's errors never reach the root.
  return beet::sequence(
      beet::named<"wait for joints">(&waitForJoints),
      beet::fallback(
          beet::named<"retry">(beet::retry(2, beet::named<"pick and place">(logged))),
          beet::named<"recover">(recover)));
}

using Tree = decltype(makeTree(std::declval<Cell>()));

static_assert(std::is_same_v<beet::input_t<Tree>, Cell>);
static_assert(std::is_same_v<beet::output_t<Tree>, Cell>);
// Asserted as GaveUp rather than never: recovering GaveUp too would make the root always succeed,
// turning the root green in the graph and the exit code into a value check.
static_assert(std::is_same_v<beet::error_t<Tree>, GaveUp>);

}  // namespace emma_behaviors
