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

#include "doctest.h"

TEST_CASE("Testing of ConstraintTreeNode class") {
  Execution e;
  Path p1;
  p1.PushBack(1);
  p1.PushBack(2);
  p1.PushBack(3);
  p1.PushBack(4);
  e.PushBack(std::make_shared<const Path>(p1));

  Path p2;
  p2.PushBack(9);
  p2.PushBack(8);
  p2.PushBack(7);
  e.PushBack(std::make_shared<const Path>(p2));

  std::map<uint64_t, std::shared_ptr<const decoupled::Conflict>> conflicts;
  std::shared_ptr<decoupled::ConstraintTreeNode> root =
      std::make_shared<decoupled::ConstraintTreeNode>(e, conflicts);

  for (Agent i = 0; i < 2; i++) {
    auto constraints = root->get_constraints(i);
    CHECK(constraints.empty());
  }

  Path p3;
  p3.PushBack(15);
  p3.PushBack(16);
  p3.PushBack(17);
  p3.PushBack(18);
  p3.PushBack(19);

  std::shared_ptr<decoupled::ConstraintTreeNode> child1 =
      std::make_shared<decoupled::ConstraintTreeNode>(
          root, decoupled::Constraint{1, 1, true}, 1,
          std::make_shared<const Path>(p3));

  CHECK(child1->get_constraints(0).empty());
  CHECK_EQ(child1->get_constraints(1).size(), 1);

  std::shared_ptr<decoupled::ConstraintTreeNode> child2 =
      std::make_shared<decoupled::ConstraintTreeNode>(
          child1, decoupled::Constraint{1, 2, true}, 1,
          std::make_shared<const Path>(p3));

  CHECK(child2->get_constraints(0).empty());
  CHECK_EQ(child2->get_constraints(1).size(), 2);

  std::shared_ptr<decoupled::ConstraintTreeNode> child3 =
      std::make_shared<decoupled::ConstraintTreeNode>(
          child2, decoupled::Constraint{1, 2, true}, 0,
          std::make_shared<const Path>(p3));

  CHECK_EQ(child3->get_constraints(0).size(), 1);
  CHECK_EQ(child3->get_constraints(1).size(), 2);
}
