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
#include <stack>
#include <vector>
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

  virtual void Split(std::list<ConstraintTreeNode>*,
                     std::shared_ptr<ConstraintTreeNode>, uint64_t) = 0;

  // TODO(arqueffe): Add support for edge collision.
  // TODO(arqueffe): Implement partial conflict computation

  void ComputeCollision(ConstraintTreeNode* ctn, uint64_t time) {
    std::shared_ptr<CollisionConflict> conflict =
        std::make_shared<CollisionConflict>();

    std::vector<bool> agent_treated =
        std::vector<bool>(instance_.nb_agents(), false);

    for (Agent a = 0; a < instance_.nb_agents(); a++) {
      if (std::find(agent_treated.begin(), agent_treated.end(), a) ==
          agent_treated.end()) {
        std::set<Agent> cluster;
        cluster.insert(a);
        Node aPos = ctn->get_path(a)->GetAtTimeOrLast(time);

        for (Agent b = a + 1; b < instance_.nb_agents(); b++) {
          if (std::find(agent_treated.begin(), agent_treated.end(), b) ==
              agent_treated.end()) {
            Node bPos = ctn->get_path(b)->GetAtTimeOrLast(time);
            if (aPos == bPos) cluster.insert(b);
          }
        }

        conflict->PushBack(cluster);
      }
    }
    if (conflict->size() == instance_.nb_agents())
      ctn->remove_conflict(time);
    else
      ctn->set_conflict(time, conflict);
  }

  void ComputeDisconnection(ConstraintTreeNode* ctn, uint64_t time) {
    std::shared_ptr<DisconnectionConflict> conflict =
        std::make_shared<DisconnectionConflict>();
    std::vector<bool> agent_treated =
        std::vector<bool>(instance_.nb_agents(), false);
    std::stack<Agent> agent_stack;
    int agent_count = 0;
    vector<bool>::iterator it;

    while ((it = find(agent_treated.begin(), agent_treated.end(), false)) !=
           agent_treated.end()) {
      agent_stack.push(distance(agent_treated.begin(), it));

      conflict->PushBack(boost::set<Agent>());
      boost::set<Agent>& cluster = conflict->back();
      while (!agent_stack.empty()) {
        Agent a = agent_stack.top();
        cluster.insert(a);
        agent_stack.pop();
        if (agent_treated[a]) continue;
        agent_treated[a] = true;
        agent_count++;

        Node aPos = ctn->get_path(a)->GetAtTimeOrLast(time);

        for (Agent b = 0; b < instance_.nb_agents(); ++b) {
          if (a != b && !agent_treated[b]) {
            Node bPos = ctn->get_path(b)->GetAtTimeOrLast(bTime);
            const std::set<Node>& neighbors =
                instance_.graph().communication().get_neighbors(aPos);

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

  void RecomputeConflicts(ConstraintTreeNode* ctn, Agent agt) {
    /** WARNING: This function assumes that
     * - ctn.get_path(a) != ctn.get_parent().get_path(a)
     * - ctn.get_path(a).size() >= ctn.get_parent().get_path(a).size()
     */
    auto prev_path = ctn->get_parent()->get_path(agt);
    auto new_path = ctn->get_path(agt);

    for (uint64_t i = 0; i < new_path->size(); i++) {
      if (*prev_path[i] == *new_path[i]) continue;

      ComputeCollision(ctn, i);
      ComputeDisconnection(ctn, i);
    }
  }

  void ComputeConflicts(ConstraintTreeNode* root) {
    /** WARNING: This function assumes that
     * - root has no parent
     */
    auto& exec = root->get_execution();
    uint64_t max = 0;
    for (Agent i = 0; i < instance_.nb_agents(); i++) {
      if (max < exec.get_path(i).size()) max = exec.get_path(i).size();
    }

    for (uint64_t i = 0; i < max; i++) {
      ComputeCollision(root, i);
      ComputeDisconnection(root, i);
    }
  }

 protected:
  void CreateChild(std::list<std::shared_ptr<ConstraintTreeNode>>* children,
                   std::shared_ptr<ConstraintTreeNode> parent, Agent agt,
                   const Constraint& c) {
    std::list<std::set<Constraint>> agt_cons = parent->get_constraints(agt);
    Path new_path = low_level_.compute(agt, agt_cons);

    if (new_path.size() == 0) return;

    auto new_path_ptr = std::make_shared<const Path>(new_path);
    auto child =
        std::make_shared<ConstraintTreeNode>(parent, c, agt, new_path_ptr);

    RecomputeConflicts(child, agt);

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
