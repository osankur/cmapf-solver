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
#include <CTNOrderingStrategy.hpp>
#include <ConflictSelectionStrategy.hpp>
#include <HighLevel.hpp>
#include <Instance.hpp>
#include <Objective.hpp>
#include <LowLevel.hpp>

namespace decoupled {

namespace high_level {

template <class GraphMove, class GraphComm>
class CBS : public HighLevel<GraphMove, GraphComm,
                             low_level::NegativeAStar<GraphMove, GraphComm>> {
 private:
  void Split(std::list<ConstraintTreeNode>*,
             std::shared_ptr<ConstraintTreeNode>, uint64_t) override {}

 public:
  CBS(const instance::Instance<GraphMove, GraphComm>& instance,
      const Objective& objective,
      const ctn_ordering::CTNOrderingStrategy& ordering_strategy,
      const ConflictSelectionStrategy& selection_strategy)
      : HighLevel(instance, objective, ordering_strategy, selection_strategy) {}
};

}  // namespace high_level

}  // namespace decoupled
