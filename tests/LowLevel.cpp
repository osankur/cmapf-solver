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
#include <InstanceLoader.hpp>
#include <LowLevel.hpp>
#include <Constraint.hpp>

#include "AppConfig.h"
#include "doctest.h"

TEST_CASE("Testing of NegativeAStar class") {
  instance::XMLInstanceLoader il(std::string(PROJECT_SOURCE_DIR) + "/tests/assets/Test1.exp",
                                 std::string(PROJECT_SOURCE_DIR) + "/tests/assets/");
  il.Load();

  decoupled::low_level::NegativeAStar<ExplicitGraph, ExplicitGraph> astar(il.instance());

  std::map<uint64_t, std::list<decoupled::Constraint>> cons;
  decoupled::Constraint cNone{0, 0, false};

  Path p1 = astar.compute(cons, cNone, 0, 1, 0);
  CHECK_EQ(p1[0], 0);
  CHECK_EQ(p1[1], 1);

  decoupled::Constraint c1{1, 1, false};

  Path p2 = astar.compute(cons, c1, 0, 1, 0);
  CHECK_EQ(p2[0], 0);
  CHECK_NE(p2[1], 1);
  CHECK_EQ(p2[2], 1);

  cons.insert(std::make_pair(1, std::list<decoupled::Constraint>{c1}));

  Path p3 = astar.compute(cons, cNone, 0, 1, 0);
  CHECK_EQ(p3[0], 0);
  CHECK_NE(p3[1], 1);
  CHECK_EQ(p3[2], 1);

  Path p4 = astar.compute(cons, cNone, 0, 1, 1);
  CHECK_EQ(p4[0], 0);
  CHECK_EQ(p4[1], 1);

  // TODO(arqueffe): Addition of more robust testing

}