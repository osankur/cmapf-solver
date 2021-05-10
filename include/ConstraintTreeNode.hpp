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
#include <map>
#include <memory>
#include <Conflict.hpp>
#include <Constraint.hpp>
#include <Execution.hpp>

namespace decoupled {

class ConstraintTreeNode
    : public std::enable_shared_from_this<ConstraintTreeNode> {
 private:
  const std::shared_ptr<ConstraintTreeNode> parent_;
  uint64_t additional_cost_;
  Execution execution_;
  std::map<uint64_t, std::shared_ptr<const Conflict>> conflicts_;

 public:
  // Root Constructor
  ConstraintTreeNode(Execution&,
                     std::map<uint64_t, std::shared_ptr<const Conflict>>);
  // Child Constructor
  ConstraintTreeNode(const std::shared_ptr<ConstraintTreeNode>,
                     const Constraint, const Agent,
                     std::shared_ptr<const Path>);
  ~ConstraintTreeNode() = default;
  ConstraintTreeNode(const ConstraintTreeNode& other) = delete;
  ConstraintTreeNode(ConstraintTreeNode&& other) = delete;
  ConstraintTreeNode& operator=(const ConstraintTreeNode& other) = delete;
  ConstraintTreeNode& operator=(ConstraintTreeNode&& other) = delete;

  void set_conflict(uint64_t, std::shared_ptr<const Conflict>);
  void remove_conflict(uint64_t);

  const Constraint constraint;
  const Agent agent;
  const std::map<uint64_t, std::shared_ptr<const Conflict>>& get_conflicts()
      const;
  std::map<uint64_t, std::list<Constraint>> get_constraints(Agent) const;
  const std::shared_ptr<const Path> get_path(Agent) const;
  const std::shared_ptr<ConstraintTreeNode> get_parent() const;
  const Execution& get_execution() const;
};

}  // namespace decoupled
