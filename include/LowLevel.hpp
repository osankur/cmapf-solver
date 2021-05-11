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

#include <limits>
#include <list>
#include <numeric>
#include <map>
#include <vector>
#include <memory>
#include <Common.hpp>
#include <Constraint.hpp>
#include <Instance.hpp>
#include <Path.hpp>
#include <boost/heap/fibonacci_heap.hpp>

namespace decoupled {

template <class GraphMove, class GraphComm>
class LowLevel {
 protected:
  const Instance<GraphMove, GraphComm> &instance_;

  bool IsNegativelyConstrained(const std::map<uint64_t, std::list<Constraint>> &cons, uint64_t time, Node n) const {
    auto found = cons.find(time);
    if (found == cons.end()) return false;

    auto &cons_time = found->second;
    for (auto &c : cons_time)
      if (c.type == false && c.node == n) return true;
    return false;
  }

 public:
  explicit LowLevel(const Instance<GraphMove, GraphComm> &instance) : instance_(instance) {}

  virtual Path compute(const std::map<uint64_t, std::list<Constraint>> &, const Constraint &, Node, Node, uint64_t) = 0;
  Path compute(const std::map<uint64_t, std::list<Constraint>> &cons, const Constraint &c, Node source, Node target) {
    return compute(cons, c, source, target, 0);
  }
};

namespace low_level {

template <class GraphMove, class GraphComm>
class NegativeAStar : public LowLevel<GraphMove, GraphComm> {
 private:
  struct AStarNode : std::enable_shared_from_this<AStarNode> {
   public:
    const Node node;
    const uint64_t time;
    std::shared_ptr<AStarNode> pred;

    AStarNode(Node n, uint64_t t, std::shared_ptr<AStarNode> p) : node(n), time(t), pred(p) {}
  };

  class HeapComparator {
   private:
    const Instance<GraphMove, GraphComm> &instance_;

   public:
    explicit HeapComparator(const Instance<GraphMove, GraphComm> &instance) : instance_(instance) {}
    // TODO(arqueffe): Add heuristic support
    bool operator()(const std::shared_ptr<AStarNode> &a, const std::shared_ptr<AStarNode> &b) const {
      return a->time > b->time;
    }
  };

  // TODO(arqueffe): Avoid reversing process
  Path RetrievePath(std::shared_ptr<AStarNode> end) const {
    std::vector<std::shared_ptr<AStarNode>> reverse_path;
    std::shared_ptr<AStarNode> current = end;
    Path p;
    while (current->pred != nullptr) {
      reverse_path.push_back(current);
      current = current->pred;
    }
    reverse_path.push_back(current);
    for (std::vector<std::shared_ptr<AStarNode>>::reverse_iterator r_it = reverse_path.rbegin();
         r_it != reverse_path.rend(); ++r_it)
      p.PushBack((*r_it)->node);
    return p;
  }

 public:
  explicit NegativeAStar(const Instance<GraphMove, GraphComm> &instance) : LowLevel(instance) {}

  Path compute(const std::map<uint64_t, std::list<Constraint>> &cons, const Constraint &c, Node source, Node target,
               uint64_t time) override {
    HeapComparator cmp(instance_);
    boost::heap::fibonacci_heap<std::shared_ptr<AStarNode>, boost::heap::compare<HeapComparator>> open(cmp);

    std::shared_ptr<AStarNode> start = std::make_shared<AStarNode>(source, time, nullptr);

    open.push(start);

    while (!open.empty()) {
      std::shared_ptr<AStarNode> current = open.top();
      open.pop();

      if (current->node == target) return RetrievePath(current);

      for (Node neighbor : instance_.graph().movement().get_neighbors(current->node)) {
        if (IsNegativelyConstrained(cons, current->time + 1, neighbor) ||
            (c.type == false && current->time + 1 == c.time && neighbor == c.node))
          continue;

        std::shared_ptr<AStarNode> next = std::make_shared<AStarNode>(neighbor, current->time + 1, current);
        open.push(next);
      }
    }
    return Path();
  }
};

}  // namespace low_level

}  // namespace decoupled
