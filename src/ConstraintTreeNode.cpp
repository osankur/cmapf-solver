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
#include <ConstraintTreeNode.hpp>

decoupled::ConstraintTreeNode::ConstraintTreeNode(
    Execution& initial_execution,
    std::map<uint64_t, std::shared_ptr<const Conflict>> initial_conflicts)
    : parent_(nullptr),
      additional_cost_(0),
      execution_(initial_execution),
      conflicts_(initial_conflicts),
      constraint(Constraint{0, 0, false}),
      agent(-2) {}

decoupled::ConstraintTreeNode::ConstraintTreeNode(
    const std::shared_ptr<ConstraintTreeNode> parent,
    const decoupled::Constraint cons, const Agent agt,
    std::shared_ptr<const Path> agt_path)
    : parent_(parent),
      additional_cost_(0),
      execution_(parent->execution_),
      conflicts_(parent->conflicts_),
      constraint(cons),
      agent(agt) {
  execution_.set_path(agt, agt_path);
}

const std::map<uint64_t, std::shared_ptr<const decoupled::Conflict>>&
decoupled::ConstraintTreeNode::get_conflicts() const {
  return conflicts_;
}

std::map<uint64_t, std::list<decoupled::Constraint>>
decoupled::ConstraintTreeNode::get_constraints(Agent agt) const {
  std::map<uint64_t, std::list<decoupled::Constraint>> constraints;

  std::shared_ptr<const decoupled::ConstraintTreeNode> cur = shared_from_this();

  while (cur != nullptr) {
    if (cur->agent == agt) {
      auto found = constraints.find(cur->constraint.time);
      if (found != constraints.end()) {
        found->second.push_back(cur->constraint);
      } else {
        constraints[cur->constraint.time] =
            std::list<decoupled::Constraint>{cur->constraint};
      }
    }
    cur = cur->parent_;
  }
  return constraints;
}
