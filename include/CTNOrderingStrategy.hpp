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

#include <memory>
#include <ConstraintTreeNode.hpp>

namespace decoupled {

class CTNOrderingStrategy {
 public:
  virtual bool operator()(const std::shared_ptr<ConstraintTreeNode> &,
                          const std::shared_ptr<ConstraintTreeNode> &) const = 0;
};

namespace ctn_ordering {

class LeastConflictStrategy : public CTNOrderingStrategy {
 public:
  bool operator()(const std::shared_ptr<ConstraintTreeNode> &first,
                  const std::shared_ptr<ConstraintTreeNode> &second) const override {
    return first->get_conflicts().size() > second->get_conflicts().size();
  }
};

}  // namespace ctn_ordering

}  // namespace decoupled
