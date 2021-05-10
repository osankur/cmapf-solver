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
#include <set>
#include <memory>
#include <CTNOrderingStrategy.hpp>
#include <ConflictSelectionStrategy.hpp>
#include <ConstraintTreeNode.hpp>
#include <Instance.hpp>
#include <LowLevel.hpp>
#include <Objective.hpp>
#include <boost/heap/fibonacci_heap.hpp>

namespace decoupled {

template <class GraphMove, class GraphComm, class LowLevel>
class HighLevel {
 private:
  const instance::Instance<GraphMove, GraphComm>& instance_;
  const Objective& objective_;
  const ctn_ordering::CTNOrderingStrategy& ordering_strategy_;
  const ConflictSelectionStrategy& selection_strategy_;

  LowLevel& low_level_;
  boost::heap::fibonacci_heap<
      std::shared_ptr<ConstraintTreeNode>,
      boost::heap::compare<ctn_ordering::CTNOrderingStrategy>>
      open_;
  virtual void split(std::list<ConstraintTreeNode>*,
                     std::shared_ptr<ConstraintTreeNode>, uint64_t) = 0;

 protected:
  void createChild(std::list<std::shared_ptr<ConstraintTreeNode>>* children,
                   std::shared_ptr<ConstraintTreeNode> parent, Agent agt,
                   const Constraint& c) {
    std::list<std::set<Constraint>> agt_cons = parent->get_constraints(agt);
    Path new_path = low_level_.compute(agt, agt_cons);
    if (new_path.size() == 0) return;
    auto new_path_ptr = std::make_shared<const Path>(new_path);
    auto child =
        std::make_shared<ConstraintTreeNode>(parent, c, agt, new_path_ptr);
    // TODO(arqueffe): recompute conflicts here !
    children.push_back(child);
  }

 public:
  HighLevel(const instance::Instance<GraphMove, GraphComm>& instance,
            const Objective& objective,
            const ctn_ordering::CTNOrderingStrategy& ordering_strategy,
            const ConflictSelectionStrategy& selection_strategy)
      : instance_(instance),
        objective_(objective),
        ordering_strategy_(ordering_strategy),
        selection_strategy_(selection_strategy) {}
};

}  // namespace decoupled
