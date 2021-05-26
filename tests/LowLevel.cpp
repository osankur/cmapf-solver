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
#include <Execution.hpp>

TEST_CASE("Testing of NegativeAStar::ComputeShortestPath") {
  instance::XMLInstanceLoader il(std::string(PROJECT_SOURCE_DIR) + "/tests/assets/Test1.exp",
                                 std::string(PROJECT_SOURCE_DIR) + "/tests/assets/");
  il.Load();

  decoupled::low_level::NegativeAStar<ExplicitGraph, ExplicitGraph> astar(il.instance());

  Path p1 = astar.ComputeShortestPath(0, 1);
  CHECK_EQ(p1[0], 0);
  CHECK_EQ(p1[1], 1);

  // TODO(arqueffe): Addition of more robust testing
}

TEST_CASE("Testing of NegativeAStar class") {
  instance::XMLInstanceLoader il(std::string(PROJECT_SOURCE_DIR) + "/tests/assets/Test1.exp",
                                 std::string(PROJECT_SOURCE_DIR) + "/tests/assets/");
  il.Load();
  decoupled::low_level::NegativeAStar<ExplicitGraph, ExplicitGraph> astar(il.instance());
  std::map<uint64_t, std::list<decoupled::Constraint>> cons;
  decoupled::Constraint cNone{0, 0, false};
  for (Agent agt = 0; agt < static_cast<Agent>(il.instance().nb_agents()); agt++) {
    Path p = astar.ComputeShortestPath(il.instance().start()[agt], il.instance().goal()[agt]);
    std::cout << p << std::endl;
    CHECK_EQ(p[0], il.instance().start()[agt]);
    CHECK_EQ(p[p.size() - 1], il.instance().goal()[agt]);
  }

  // TODO(arqueffe): Addition of robust testing
}
