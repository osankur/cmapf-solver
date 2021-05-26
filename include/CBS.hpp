/* Copyright (c) 2021 Arthur Queffelec
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 */
#pragma once

#include <list>
#include <memory>
#include <set>
#include <CTNOrderingStrategy.hpp>
#include <ConflictSelectionStrategy.hpp>
#include <HighLevel.hpp>
#include <Instance.hpp>
#include <Objective.hpp>
#include <LowLevel.hpp>
#include <Conflict.hpp>

namespace decoupled {

namespace high_level {

template <class GraphMove, class GraphComm>
class CBS : public HighLevel<GraphMove, GraphComm> {
 private:
  using priority_queue =
      boost::heap::fibonacci_heap<std::shared_ptr<ConstraintTreeNode>, boost::heap::compare<CTNOrderingStrategy>>;

  void Split(priority_queue* children, std::shared_ptr<ConstraintTreeNode> ctn, uint64_t time) override {
    const std::shared_ptr<const Conflict> conflict = ctn->get_conflicts().at(time);
    const std::shared_ptr<const CollisionConflict> cast_conflict =
        std::dynamic_pointer_cast<const CollisionConflict>(conflict);
    const std::set<Agent>& first_conflict = cast_conflict->back();
    for (Agent agt : first_conflict) {
      Node pos_conflict = ctn->get_path(agt)->GetAtTimeOrLast(time);
      this->CreateChild(children, ctn, agt, Constraint{pos_conflict, time, false});
    }
  }

 public:
  CBS(const Instance<GraphMove, GraphComm>& instance, const Objective& objective,
      const CTNOrderingStrategy& ordering_strategy, const ConflictSelectionStrategy& selection_strategy)
      : decoupled::HighLevel<GraphMove, GraphComm>(
            instance, objective, ordering_strategy, selection_strategy,
            std::make_unique<low_level::NegativeAStar<GraphMove, GraphComm>>(instance)) {}
};

}  // namespace high_level

}  // namespace decoupled
