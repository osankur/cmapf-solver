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
#include <map>
#include <stack>
#include <vector>
#include <utility>
#include <unordered_set>
#include <CTNOrderingStrategy.hpp>
#include <FloydWarshall.hpp>
#include <ConflictSelectionStrategy.hpp>
#include <ConstraintTreeNode.hpp>
#include <Instance.hpp>
#include <LowLevel.hpp>
#include <Objective.hpp>
#include <Solver.hpp>
#include <boost/heap/fibonacci_heap.hpp>

namespace decoupled {

template <class GraphMove, class GraphComm>
class AStar : public Solver<GraphMove, GraphComm> {
 private:
  struct AStarNode {
    const Configuration config;
    const size_t time;
  };

  struct AStarNodePtrEqual {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
      return a->config == b->config && a->time == b->time;
    }
  };

  struct AStarNodePtrHash {
    size_t operator()(const std::shared_ptr<AStarNode>& that) const {
      return std::hash<Node>()(that->node) ^ std::hash<uint64_t>()(that->time);
    }
  };

 public:
  AStar(const Instance<GraphMove, GraphComm>& instance, const Objective& objective, int depth)
      : Solver<GraphMove, GraphComm>(instance, objective) {}

  bool StepCompute() override {}
};

}  // namespace decoupled
