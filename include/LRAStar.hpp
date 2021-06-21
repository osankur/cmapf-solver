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
#include <CTNOrderingStrategy.hpp>
#include <ConflictSelectionStrategy.hpp>
#include <ConstraintTreeNode.hpp>
#include <Instance.hpp>
#include <LowLevel.hpp>
#include <Objective.hpp>
#include <Solver.hpp>
#include <boost/heap/fibonacci_heap.hpp>

namespace decoupled {

template <class GraphMove, class GraphComm>
class LRAStar : public Solver<GraphMove, GraphComm> {
 private:
  class LRANode {
   private:
    Execution execution_;

   public:
    LRANode() : execution_(){};
    LRANode(Agent agt, std::shared_ptr<const Path> path) : execution_() { execution_.set_path(agt, path); };
    LRANode(const std::shared_ptr<LRANode>& node, Agent agt, std::shared_ptr<const Path> path)
        : execution_(node->execution_) {
      execution_.set_path(agt, path);
    };
    ~LRANode(){};
    LRANode(const LRANode& other) : execution_(other.execution_){};
    LRANode(LRANode&& other) : execution_(other.execution_){};
    LRANode& operator=(const LRANode& other) { execution_ = other.execution_; };
    LRANode& operator=(LRANode&& other) { execution_ = other.execution_; };

    const std::shared_ptr<const Path> path(Agent a) const { return execution_.get_path(a); }
    bool IsAgentSet(Agent a) const {
      return a < static_cast<Agent>(execution_.size()) && execution_.get_path(a) != nullptr;
    }
    const Execution& get_execution() const { return execution_; }
  };

  class LRANodePtrComparator {
   private:
    const Instance<GraphMove, GraphComm>& instance_;
    const Objective& objective_;

   public:
    LRANodePtrComparator(const Instance<GraphMove, GraphComm>& instance, const Objective& objective)
        : instance_(instance), objective_(objective) {}
    ~LRANodePtrComparator() = default;
    LRANodePtrComparator(const LRANodePtrComparator& other) = default;
    LRANodePtrComparator(LRANodePtrComparator&& other) = default;
    LRANodePtrComparator& operator=(const LRANodePtrComparator& other) = default;
    LRANodePtrComparator& operator=(LRANodePtrComparator&& other) = default;

    bool operator()(const std::shared_ptr<LRANode>& first, const std::shared_ptr<LRANode>& second) const {
      size_t lengthFirst = objective_.cost(first->get_execution());
      size_t lengthSecond = objective_.cost(second->get_execution());
      size_t heuristicFirst = 0;
      size_t heuristicSecond = 0;
      for (Agent agt = 0; static_cast<size_t>(agt) < instance_.nb_agents(); agt++) {
        if (!first->IsAgentSet(agt))
          heuristicFirst += instance_.graph().movement().get_distance(instance_.start()[agt], instance_.goal()[agt]);
        if (!second->IsAgentSet(agt))
          heuristicSecond += instance_.graph().movement().get_distance(instance_.start()[agt], instance_.goal()[agt]);
      }
      return lengthFirst + heuristicFirst < lengthSecond + heuristicSecond;
    }
  };

  using priority_queue =
      boost::heap::fibonacci_heap<std::shared_ptr<LRANode>, boost::heap::compare<LRANodePtrComparator>>;
  priority_queue open_;

  bool IsLRANodeDone(const std::shared_ptr<LRANode> node) const {
    for (Agent agt = 0; static_cast<size_t>(agt) < this->instance_.nb_agents(); agt++) {
      if (!node->IsAgentSet(agt)) return false;
    }
    return true;
  }

  struct AStarNode : std::enable_shared_from_this<AStarNode> {
   public:
    const Node node;
    const uint64_t time;
    std::shared_ptr<AStarNode> pred;

    AStarNode(Node n, uint64_t t, std::shared_ptr<AStarNode> p) : node(n), time(t), pred(p) {}
  };

  struct AStarNodePtrEqual {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
      return a->node == b->node && a->time && b->time;
    }
  };

  struct AStarNodePtrHash {
    size_t operator()(const std::shared_ptr<AStarNode>& that) const {
      return std::hash<Node>()(that->node) ^ std::hash<uint64_t>()(that->time);
    }
  };

  class HeapComparator {
   private:
    const Instance<GraphMove, GraphComm>& instance_;
    const Node& target_;

   public:
    explicit HeapComparator(const Instance<GraphMove, GraphComm>& instance, const Node& target)
        : instance_(instance), target_(target) {}
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
      return a->time + instance_.graph().movement().get_distance(a->node, target_) >
             b->time + instance_.graph().movement().get_distance(b->node, target_);
    }
  };

  Path ComputeInitialPath(Agent agt) {
    decoupled::low_level::NegativeAStar<GraphMove, GraphComm> nastar(this->instance_);
    return nastar.ComputeShortestPath(this->instance_.start()[agt], this->instance_.goal()[agt]);
  }

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
    for (auto r_it = reverse_path.rbegin(); r_it != reverse_path.rend(); ++r_it) p.PushBack((*r_it)->node);
    return p;
  }

  Path ComputeConnectedPath(const std::shared_ptr<LRANode>& lranode, Agent agt) {
    HeapComparator cmp(this->instance_, this->instance_.goal()[agt]);
    boost::heap::fibonacci_heap<std::shared_ptr<AStarNode>, boost::heap::compare<HeapComparator>> open(cmp);
    std::unordered_set<std::shared_ptr<AStarNode>, AStarNodePtrHash, AStarNodePtrEqual> closed;

    std::shared_ptr<AStarNode> start = std::make_shared<AStarNode>(this->instance_.start()[agt], 0, nullptr);
    open.push(start);
    closed.insert(start);
    while (!open.empty()) {
      std::shared_ptr<AStarNode> current = open.top();
      open.pop();

      if (current->node == this->instance_.goal()[agt]) return RetrievePath(current);

      for (Agent agtConn = 0; static_cast<size_t>(agtConn) < this->instance_.nb_agents(); agtConn++) {
        if (!lranode->IsAgentSet(agtConn)) continue;

        for (Node neighbor : this->instance_.graph().communication().get_neighbors(
                 lranode->path(agtConn)->GetAtTimeOrLast(current->time + 1))) {
          auto& currentNeighbors = this->instance_.graph().movement().get_neighbors(current->node);
          if (currentNeighbors.find(neighbor) == currentNeighbors.end()) continue;

          std::shared_ptr<AStarNode> next = std::make_shared<AStarNode>(neighbor, current->time + 1, current);

          if (closed.find(next) == closed.end()) {
            open.push(next);
            closed.insert(next);
          }
        }
      }
    }
    return Path();
  }

 public:
  LRAStar(const Instance<GraphMove, GraphComm>& instance, const Objective& objective)
      : Solver<GraphMove, GraphComm>(instance, objective), open_(LRANodePtrComparator(instance, objective)) {
    for (Agent agt = 0; static_cast<size_t>(agt) < this->instance_.nb_agents(); agt++) {
      Path p = ComputeInitialPath(agt);
      if (p.size() > 0) open_.push(std::make_shared<LRANode>(agt, std::make_shared<const Path>(p)));
    }
  }

  bool StepCompute() override {
    if (open_.empty()) return true;

    auto top = open_.top();
    open_.pop();

    if (IsLRANodeDone(top)) {
      this->execution_ = top->get_execution();
      return true;
    }

    for (Agent agt = 0; static_cast<size_t>(agt) < this->instance_.nb_agents(); agt++) {
      if (!top->IsAgentSet(agt)) {
        Path p = ComputeConnectedPath(top, agt);
        if (p.size() > 0) open_.push(std::make_shared<LRANode>(top, agt, std::make_shared<const Path>(p)));
      }
    }

    return false;
  }
};

}  // namespace decoupled
