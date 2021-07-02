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
class CAStar : public Solver<GraphMove, GraphComm> {
 private:
  class CANode {
   private:
    Execution execution_;

   public:
    CANode() : execution_() {}
    CANode(Agent agt, std::shared_ptr<const Path> path) : execution_() { execution_.set_path(agt, path); }
    CANode(const std::shared_ptr<CANode>& node, Agent agt, std::shared_ptr<const Path> path)
        : execution_(node->execution_) {
      execution_.set_path(agt, path);
    }
    ~CANode() {}
    CANode(const CANode& other) : execution_(other.execution_) {}
    CANode(CANode&& other) : execution_(other.execution_) {}
    CANode& operator=(const CANode& other) { execution_ = other.execution_; }
    CANode& operator=(CANode&& other) { execution_ = other.execution_; }

    const std::shared_ptr<const Path> path(Agent a) const { return execution_.get_path(a); }
    bool IsAgentSet(Agent a) const {
      return a < static_cast<Agent>(execution_.size()) && execution_.get_path(a) != nullptr;
    }
    const Execution& get_execution() const { return execution_; }
  };

  class CANodePtrComparator {
   private:
    const Instance<GraphMove, GraphComm>& instance_;
    const Objective& objective_;
    const FloydWarshall<GraphMove, GraphComm>& fw_;
    const Configuration& start_;

   public:
    CANodePtrComparator(const Instance<GraphMove, GraphComm>& instance, const Objective& objective,
                        const FloydWarshall<GraphMove, GraphComm>& fw, const Configuration& start)
        : instance_(instance), objective_(objective), fw_(fw), start_(start) {}
    ~CANodePtrComparator() = default;
    CANodePtrComparator(const CANodePtrComparator& other) = default;
    CANodePtrComparator(CANodePtrComparator&& other) = default;
    CANodePtrComparator& operator=(const CANodePtrComparator& other) = default;
    CANodePtrComparator& operator=(CANodePtrComparator&& other) = default;

    bool operator()(const std::shared_ptr<CANode>& first, const std::shared_ptr<CANode>& second) const {
      size_t lengthFirst = objective_.cost(first->get_execution());
      size_t lengthSecond = objective_.cost(second->get_execution());
      size_t heuristicFirst = 0;
      size_t heuristicSecond = 0;
      for (Agent agt = 0; static_cast<size_t>(agt) < instance_.nb_agents(); agt++) {
        if (!first->IsAgentSet(agt)) heuristicFirst += fw_.GetShortestPathSize(start_.at(agt), instance_.goal()[agt]);
        if (!second->IsAgentSet(agt)) heuristicSecond += fw_.GetShortestPathSize(start_.at(agt), instance_.goal()[agt]);
      }
      return lengthFirst + heuristicFirst < lengthSecond + heuristicSecond;
    }
  };

  using priority_queue =
      boost::heap::fibonacci_heap<std::shared_ptr<CANode>, boost::heap::compare<CANodePtrComparator>>;
  FloydWarshall<GraphMove, GraphComm> fw_;
  priority_queue open_;
  const int depth_;
  std::vector<Path> exec_;
  Configuration current_;

  bool IsWCANodeDone(const std::shared_ptr<CANode> node) const {
    for (Agent agt = 0; static_cast<size_t>(agt) < this->instance_.nb_agents(); agt++) {
      if (!node->IsAgentSet(agt) || node->path(agt)->at(node->path(agt)->size() - 1) != this->instance_.goal()[agt])
        return false;
    }
    return true;
  }

  bool IsCANodeDone(const std::shared_ptr<CANode> node) const {
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
      return a->node == b->node && a->time == b->time;
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
    const FloydWarshall<GraphMove, GraphComm>& fw_;
    const int depth_;

   public:
    explicit HeapComparator(const Instance<GraphMove, GraphComm>& instance, const Node& target,
                            const FloydWarshall<GraphMove, GraphComm>& fw, int depth)
        : instance_(instance), target_(target), fw_(fw), depth_(depth) {}
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
      size_t sizeA = a->time + fw_.GetShortestPathSize(a->node, target_);
      size_t sizeB = b->time + fw_.GetShortestPathSize(b->node, target_);
      return sizeA > sizeB;
      // instance_.graph().movement().get_distance(a->node, target_) >
      // instance_.graph().movement().get_distance(b->node, target_);
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
    for (auto r_it = reverse_path.rbegin(); r_it != reverse_path.rend(); ++r_it) p.PushBack((*r_it)->node);
    return p;
  }

  Path ComputeConnectedPath(const std::shared_ptr<CANode>& canode, Agent agt) {
    HeapComparator cmp(this->instance_, this->instance_.goal()[agt], fw_, depth_);
    boost::heap::fibonacci_heap<std::shared_ptr<AStarNode>, boost::heap::compare<HeapComparator>> open(cmp);
    std::unordered_set<std::shared_ptr<AStarNode>, AStarNodePtrHash, AStarNodePtrEqual> closed;

    std::shared_ptr<AStarNode> start = std::make_shared<AStarNode>(current_.at(agt), 0, nullptr);
    open.push(start);
    closed.insert(start);
    while (!open.empty()) {
      std::shared_ptr<AStarNode> current = open.top();
      open.pop();

      if (current->node == this->instance_.goal()[agt] || current->time >= depth_ - 1) return RetrievePath(current);

      const auto& currentNeighbors = this->instance_.graph().movement().get_neighbors(current->node);

      for (size_t agtOther = 0; agtOther < this->instance_.nb_agents(); agtOther++) {
        if (!canode->IsAgentSet(agtOther)) continue;

        const auto& agtConn = this->instance_.graph().communication().get_neighbors(
            canode->path(agtOther)->GetAtTimeOrLast(current->time + 1));

        for (const auto& neighbour : currentNeighbors) {
          if (agtConn.find(neighbour) == agtConn.end()) continue;

          std::shared_ptr<AStarNode> next = std::make_shared<AStarNode>(neighbour, current->time + 1, current);

          if (closed.find(next) == closed.end()) {
            open.push(next);
            closed.insert(next);
          }
        }
      }
    }
    return Path();
  }

  void Initialize() {
    open_.clear();
    for (Agent agt = 0; static_cast<size_t>(agt) < this->instance_.nb_agents(); agt++) {
      Path p = fw_.ComputeShortestPath(current_.at(agt), this->instance_.goal()[agt]);
      if (depth_ > -1 && p.size() > depth_) {
        p.Resize(depth_ + 1);
      }
      if (p.size() > 0) open_.push(std::make_shared<CANode>(agt, std::make_shared<const Path>(p)));
    }
  }

 public:
  CAStar(const Instance<GraphMove, GraphComm>& instance, const Objective& objective, int depth)
      : Solver<GraphMove, GraphComm>(instance, objective),
        fw_(instance),
        open_(CANodePtrComparator(instance, objective, fw_, instance.start())),
        depth_(depth),
        exec_(instance.nb_agents()),
        current_(instance.start()) {
    fw_.Compute();
    Initialize();
  }

  bool StepCompute() override {
    if (open_.empty()) return true;

    auto top = open_.top();
    open_.pop();

    if (IsCANodeDone(top)) {
      if (IsWCANodeDone(top)) {
        for (size_t agt = 0; agt < top->get_execution().size(); agt++) {
          for (size_t t = 1; t < top->get_execution().get_path(agt)->size(); t++) {
            exec_[agt].PushBack(top->get_execution().get_path(agt)->at(t));
          }
          this->execution_.PushBack(std::make_shared<Path>(exec_[agt]));
        }
        return true;
      }
      for (size_t agt = 0; agt < top->get_execution().size(); agt++) {
        for (size_t t = exec_[agt].size() == 0 ? 0 : 1; t < top->get_execution().get_path(agt)->size(); t++) {
          exec_[agt].PushBack(top->get_execution().get_path(agt)->at(t));
        }
      }
      size_t max_path = 0;
      for (size_t agt = 0; agt < top->get_execution().size(); agt++) {
        size_t size_path = top->get_execution().get_path(agt)->size();
        if (max_path < size_path) max_path = size_path;
      }
      current_ = top->get_execution().get_configuration(max_path);
      Initialize();
    }

    for (Agent agt = 0; static_cast<size_t>(agt) < this->instance_.nb_agents(); agt++) {
      if (!top->IsAgentSet(agt)) {
        Path p = ComputeConnectedPath(top, agt);
        if (p.size() > 0) open_.push(std::make_shared<CANode>(top, agt, std::make_shared<const Path>(p)));
      }
    }

    return false;
  }
};

}  // namespace decoupled
