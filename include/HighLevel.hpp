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
#include <Logger.hpp>
#include <boost/heap/fibonacci_heap.hpp>

namespace decoupled {

struct BypassException : public std::exception {
  std::shared_ptr<ConstraintTreeNode> child;
};

template <class GraphMove, class GraphComm>
class HighLevel : public Solver<GraphMove, GraphComm> {
 private:
  const CTNOrderingStrategy& ordering_strategy_;
  const ConflictSelectionStrategy& selection_strategy_;

  std::unique_ptr<LowLevel<GraphMove, GraphComm>> low_level_;
  using priority_queue =
      boost::heap::fibonacci_heap<std::shared_ptr<ConstraintTreeNode>, boost::heap::compare<CTNOrderingStrategy>>;
  priority_queue open_;

  virtual void Split(priority_queue*, std::shared_ptr<ConstraintTreeNode>, uint64_t) = 0;

  // TODO(arqueffe): Add support for edge collision.
  // TODO(arqueffe): Implement partial conflict computation

  void ComputeCollision(std::shared_ptr<ConstraintTreeNode> ctn, uint64_t time) {
    std::shared_ptr<CollisionConflict> conflict = std::make_shared<CollisionConflict>();

    std::vector<bool> agent_treated = std::vector<bool>(this->instance_.nb_agents(), false);

    for (Agent a = 0; a < static_cast<Agent>(this->instance_.nb_agents()); a++) {
      if (!agent_treated[a]) {
        std::set<Agent> cluster;
        cluster.insert(a);
        Node aPos = ctn->get_path(a)->GetAtTimeOrLast(time);

        for (Agent b = a + 1; b < static_cast<Agent>(this->instance_.nb_agents()); b++) {
          if (std::find(agent_treated.begin(), agent_treated.end(), b) == agent_treated.end()) {
            Node bPos = ctn->get_path(b)->GetAtTimeOrLast(time);
            if (aPos == bPos) cluster.insert(b);
          }
        }
        if (cluster.size() > 1) conflict->PushBack(cluster);
      }
    }
    if (conflict->size() == 0)
      ctn->remove_conflict(time);
    else
      ctn->set_conflict(time, conflict);
  }

  void ComputeDisconnection(std::shared_ptr<ConstraintTreeNode> ctn, uint64_t time) {
    std::shared_ptr<DisconnectionConflict> conflict = std::make_shared<DisconnectionConflict>();
    std::vector<bool> agent_treated = std::vector<bool>(this->instance_.nb_agents(), false);
    std::stack<Agent> agent_stack;
    int agent_count = 0;
    std::vector<bool>::iterator it;

    while ((it = find(agent_treated.begin(), agent_treated.end(), false)) != agent_treated.end()) {
      agent_stack.push(distance(agent_treated.begin(), it));

      conflict->PushBack(std::set<Agent>());
      std::set<Agent>& cluster = conflict->back();
      while (!agent_stack.empty()) {
        Agent a = agent_stack.top();
        cluster.insert(a);
        agent_stack.pop();
        if (agent_treated[a]) continue;
        agent_treated[a] = true;
        agent_count++;

        Node aPos = ctn->get_path(a)->GetAtTimeOrLast(time);

        for (Agent b = 0; b < static_cast<Agent>(this->instance_.nb_agents()); ++b) {
          if (a != b && !agent_treated[b]) {
            Node bPos = ctn->get_path(b)->GetAtTimeOrLast(b);
            const auto& neighbors = this->instance_.graph().communication().get_neighbors(aPos);

            if (neighbors.find(bPos) != neighbors.end()) {
              agent_stack.push(b);
            }
          }
        }
      }
    }
    if (conflict->size() == 1)
      ctn->remove_conflict(time);
    else
      ctn->set_conflict(time, conflict);
  }

  void RecomputeConflicts(std::shared_ptr<ConstraintTreeNode> ctn, Agent agt) {
    /** WARNING: This function assumes that
     * - ctn.get_path(a) != ctn.get_parent().get_path(a)
     * - ctn.get_path(a).size() >= ctn.get_parent().get_path(a).size()
     */
    auto prev_path = ctn->get_parent()->get_path(agt);
    auto new_path = ctn->get_path(agt);

    for (uint64_t i = 0; i < prev_path->size(); i++) {
      if ((*prev_path)[i] == (*new_path)[i]) continue;

      // ComputeCollision(ctn, i);
      ComputeDisconnection(ctn, i);
    }
    for (uint64_t i = prev_path->size() - 1; i < new_path->size(); i++) {
      // ComputeCollision(ctn, i);
      ComputeDisconnection(ctn, i);
    }
  }

  void ComputeConflicts(std::shared_ptr<ConstraintTreeNode> root) {
    /** WARNING: This function assumes that
     * - root has no parent
     */
    auto& exec = root->get_execution();
    uint64_t max = 0;
    for (Agent i = 0; i < static_cast<Agent>(this->instance_.nb_agents()); i++) {
      if (max < exec.get_path(i)->size()) max = exec.get_path(i)->size();
    }

    for (uint64_t i = 0; i < max; i++) {
      // ComputeCollision(root, i);
      ComputeDisconnection(root, i);
    }
  }

 protected:
  void CreateChild(priority_queue* children, std::shared_ptr<ConstraintTreeNode> parent, Agent agt,
                   const Constraint& c) {
    std::map<uint64_t, std::list<Constraint>> agt_cons = parent->get_constraints(agt);
    Path new_path =
        low_level_->ComputeConstrainedPath(agt_cons, c, this->instance_.start()[agt], this->instance_.goal()[agt]);

    if (new_path.size() == 0) return;

    auto new_path_ptr = std::make_shared<const Path>(new_path);
    auto child_ptr = std::make_shared<ConstraintTreeNode>(parent, c, agt, new_path_ptr);

    RecomputeConflicts(child_ptr, agt);

    // TODO(arqueffe): Count conflicts size depending on the type
    if (child_ptr->get_path(agt)->size() == parent->get_path(agt)->size() &&
        child_ptr->get_conflicts().size() < parent->get_conflicts().size()) {
      BypassException bp;
      bp.child = child_ptr;
      throw bp;
    }

    children->push(child_ptr);
  }

 public:
  HighLevel(const Instance<GraphMove, GraphComm>& instance, const Objective& objective,
            const CTNOrderingStrategy& ordering_strategy, const ConflictSelectionStrategy& selection_strategy,
            std::unique_ptr<LowLevel<GraphMove, GraphComm>> low_level)
      : Solver<GraphMove, GraphComm>(instance, objective),
        ordering_strategy_(ordering_strategy),
        selection_strategy_(selection_strategy),
        low_level_(std::move(low_level)),
        open_(ordering_strategy) {
    Execution start_exec;
    for (Agent agt = 0; agt < static_cast<Agent>(this->instance_.nb_agents()); agt++) {
      Path p = low_level_->ComputeShortestPath(this->instance_.start()[agt], this->instance_.goal()[agt]);
      start_exec.PushBack(std::make_shared<const Path>(p));
    }
    auto start_node = std::make_shared<ConstraintTreeNode>(start_exec);
    ComputeConflicts(start_node);
    open_.push(start_node);
  }

  bool StepCompute() override {
    std::shared_ptr<ConstraintTreeNode> top = open_.top();
    open_.pop();

    if (top->get_conflicts().size() == 0) {
      this->execution_ = top->get_execution();
      return true;
    }

    uint64_t conflict_time = selection_strategy_.SelectConflict(top->get_conflicts());

    // Using fibo for storage of children as merge is O(1)
    priority_queue children(ordering_strategy_);

    try {
      Split(&children, top, conflict_time);
    } catch (const BypassException& bp) {
      children.clear();
      children.push(std::make_shared<ConstraintTreeNode>(top, bp.child));
    }

    open_.merge(children);

    return false;
  }
};

}  // namespace decoupled
