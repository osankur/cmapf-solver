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
#include <Objective.hpp>

//#include "doctest.h"

#include <doctest/doctest.h>
TEST_CASE("Testing of MaxObjective class") {
  Execution e;

  MaxObjective max;

  CHECK_EQ(max.cost(e), 0);

  Path p1;
  p1.PushBack(1);
  p1.PushBack(2);
  p1.PushBack(3);
  p1.PushBack(4);
  e.PushBack(std::make_shared<const Path>(p1));

  CHECK_EQ(max.cost(e), 4);

  Path p2;
  p2.PushBack(9);
  p2.PushBack(8);
  p2.PushBack(7);
  e.PushBack(std::make_shared<const Path>(p2));

  CHECK_EQ(max.cost(e), 4);

  Path p3;
  p3.PushBack(15);
  p3.PushBack(16);
  p3.PushBack(17);
  p3.PushBack(18);
  p3.PushBack(19);
  e.set_path(0, std::make_shared<const Path>(p3));

  CHECK_EQ(max.cost(e), 5);

  Path p4;
  p4.PushBack(15);
  p4.PushBack(16);
  p4.PushBack(17);
  p4.PushBack(18);
  p4.PushBack(19);
  p4.PushBack(20);
  e.PushBack(std::make_shared<const Path>(p4));

  CHECK_EQ(max.cost(e), 6);
}

TEST_CASE("Testing of SumObjective class") {
  Execution e;

  SumObjective sum;

  CHECK_EQ(sum.cost(e), 0);

  Path p1;
  p1.PushBack(1);
  p1.PushBack(2);
  p1.PushBack(3);
  p1.PushBack(4);
  e.PushBack(std::make_shared<const Path>(p1));

  CHECK_EQ(sum.cost(e), 4);

  Path p2;
  p2.PushBack(9);
  p2.PushBack(8);
  p2.PushBack(7);
  e.PushBack(std::make_shared<const Path>(p2));

  CHECK_EQ(sum.cost(e), 7);

  Path p3;
  p3.PushBack(15);
  p3.PushBack(16);
  p3.PushBack(17);
  p3.PushBack(18);
  p3.PushBack(19);
  e.set_path(0, std::make_shared<const Path>(p3));

  CHECK_EQ(sum.cost(e), 8);

  Path p4;
  p4.PushBack(15);
  p4.PushBack(16);
  p4.PushBack(17);
  p4.PushBack(18);
  p4.PushBack(19);
  p4.PushBack(20);
  e.PushBack(std::make_shared<const Path>(p4));

  CHECK_EQ(sum.cost(e), 14);
}
