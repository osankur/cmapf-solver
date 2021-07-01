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
#include <queue>
#include <CTNOrderingStrategy.hpp>
#include <ConflictSelectionStrategy.hpp>
#include <ConstraintTreeNode.hpp>
#include <Instance.hpp>
#include <LowLevel.hpp>
#include <Objective.hpp>
#include <Solver.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <FloydWarshall.hpp>

namespace coupled {

template <class GraphMove, class GraphComm>
class DFS : public Solver<GraphMove, GraphComm> {
 private:
  struct ConfigurationPtrEqual {
    bool operator()(const std::shared_ptr<Configuration>& a, const std::shared_ptr<Configuration>& b) const {
      if (a->size() != b->size()) return false;

      for (size_t index = 0; index < a->size(); index++)
        if (a->at(index) != b->at(index)) return false;
      return true;
    }
  };

  struct ConfigurationPtrHash {
    size_t operator()(const std::shared_ptr<Configuration>& that) const {
      size_t h = 0;
      for (size_t index = 0; index < that->size(); index++) h += that->at(index) ^ index;

      return h;
    }
  };

  class HeapComparator {
   private:
    const Instance<GraphMove, GraphComm>& instance_;
    const std::shared_ptr<Configuration> config_;
    FloydWarshall<GraphMove, GraphComm> floydwarshall_;

   public:
    explicit HeapComparator(const Instance<GraphMove, GraphComm>& instance, const std::shared_ptr<Configuration> config)
        : instance_(instance), config_(config), floydwarshall_(instance) {
      floydwarshall_.Compute();
    }

    bool operator()(const std::shared_ptr<Configuration>& a, const std::shared_ptr<Configuration>& b) {
      size_t costA = 0;
      size_t costB = 0;
      costA += a->size();
      costB += b->size();
      for (size_t agt = 0; agt < config_->size(); agt++) {
        if (agt < a->size()) costA += 1 + floydwarshall_.GetShortestPathSize(a->at(agt), instance_.goal()[agt]);
        /*instance_.graph().movement().get_distance(config_->at(agt), a->at(agt)) +
        instance_.graph().movement().get_distance(a->at(agt), instance_.goal()[agt]);*/
        else
          costA += floydwarshall_.GetShortestPathSize(config_->at(agt), instance_.goal()[agt]);
        // instance_.graph().movement().get_distance(config_->at(agt), instance_.goal()[agt]);
        if (agt < b->size()) costB += 1 + floydwarshall_.GetShortestPathSize(b->at(agt), instance_.goal()[agt]);
        /*instance_.graph().movement().get_distance(config_->at(agt), b->at(agt)) +
        instance_.graph().movement().get_distance(b->at(agt), instance_.goal()[agt]);*/
        else
          costB += floydwarshall_.GetShortestPathSize(config_->at(agt), instance_.goal()[agt]);
        // instance_.graph().movement().get_distance(config_->at(agt), instance_.goal()[agt]);
      }
      return costA > costB;
    }
  };

  std::vector<std::shared_ptr<Configuration>> exec_;
  std::unordered_set<std::shared_ptr<Configuration>, ConfigurationPtrHash, ConfigurationPtrEqual> closed_;

  bool IsGoal(const std::shared_ptr<Configuration>& config) {
    if (config->size() != this->instance_.nb_agents()) return false;
    for (size_t agt = 0; agt < config->size(); agt++)
      if (config->at(agt) != this->instance_.goal()[agt]) return false;
    return true;
  }

  bool IsConfigurationConnected(const std::shared_ptr<Configuration>& config) {
    std::vector<bool> agent_treated = std::vector<bool>(this->instance_.nb_agents(), false);
    std::stack<Agent> agent_stack;
    size_t agent_count = 0;
    std::vector<bool>::iterator it;

    while ((it = find(agent_treated.begin(), agent_treated.end(), false)) != agent_treated.end()) {
      agent_stack.push(distance(agent_treated.begin(), it));

      while (!agent_stack.empty()) {
        Agent a = agent_stack.top();
        agent_stack.pop();

        if (agent_treated[a]) continue;

        agent_treated[a] = true;
        agent_count++;

        Node aPos = config->at(a);

        for (Agent b = 0; b < static_cast<Agent>(this->instance_.nb_agents()); ++b) {
          if (a != b && !agent_treated[b]) {
            Node bPos = config->at(b);
            const auto& neighbors = this->instance_.graph().communication().get_neighbors(aPos);

            if (neighbors.find(bPos) != neighbors.end()) {
              agent_stack.push(b);
            }
          }
        }
      }
    }
    return agent_count == this->instance_.nb_agents();
  }

  std::shared_ptr<Configuration> FindBestChild(const std::shared_ptr<Configuration> config) {
    HeapComparator cmp(this->instance_, config);
    std::priority_queue<std::shared_ptr<Configuration>, std::vector<std::shared_ptr<Configuration>>, HeapComparator>
        open(cmp);
    std::shared_ptr<Configuration> start = std::make_shared<Configuration>();
    open.push(start);
    while (!open.empty()) {
      std::shared_ptr<Configuration> current = open.top();
      open.pop();

      if (current->size() == this->instance_.nb_agents()) {
        if (IsConfigurationConnected(current) && closed_.find(current) == closed_.end())
          return current;
        else
          continue;
      }

      for (Node neighbor : this->instance_.graph().movement().get_neighbors(config->at(current->size()))) {
        std::shared_ptr<Configuration> next = std::make_shared<Configuration>();
        for (Agent agt = 0; agt < static_cast<Agent>(current->size()); agt++) next->PushBack(current->at(agt));

        next->PushBack(neighbor);
        open.push(next);
      }
    }
    return nullptr;
  }

 public:
  DFS(const Instance<GraphMove, GraphComm>& instance, const Objective& objective)
      : Solver<GraphMove, GraphComm>(instance, objective), exec_() {
    std::shared_ptr<Configuration> start = std::make_shared<Configuration>();
    for (size_t agt = 0; agt < instance.start().size(); agt++) start->PushBack(instance.start()[agt]);
    exec_.push_back(start);
    closed_.insert(start);
  }

  bool StepCompute() override {
    if (exec_.empty()) return true;
    if (IsGoal(exec_.back())) {
      for (size_t agt = 0; agt < this->instance_.nb_agents(); agt++) {
        std::shared_ptr<Path> p_agt = std::make_shared<Path>();
        for (size_t t = 0; t < exec_.size(); t++) {
          p_agt->PushBack(exec_[t]->at(agt));
        }
        this->execution_.set_path(agt, p_agt);
      }
      return true;
    }

    std::shared_ptr<Configuration> config = FindBestChild(exec_.back());
    closed_.insert(config);
    if (config == nullptr)
      exec_.pop_back();
    else
      exec_.push_back(config);
    return false;
  }
};

}  // namespace coupled
