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
#include <MAS.hpp>
#include <InstanceLoader.hpp>
#include <Objective.hpp>

#include "AppConfig.h"
#include "doctest.h"

TEST_CASE("Testing of MAS class") {
  instance::XMLInstanceLoader il(std::string(PROJECT_SOURCE_DIR) + "/tests/assets/Test1.exp",
                                 std::string(PROJECT_SOURCE_DIR) + "/tests/assets/");
  il.Load();

  SumObjective obj;

  decoupled::MAS<ExplicitGraph, ExplicitGraph> mas(il.instance(), obj);

  Execution e = mas.Compute();
  std::cout << e << std::endl;
  for (Agent agt = 0; agt < static_cast<Agent>(il.instance().nb_agents()); agt++) {
    auto p = e.get_path(agt);
    CHECK_EQ(p->GetAtTimeOrLast(0), il.instance().start()[agt]);
    CHECK_EQ(p->GetAtTimeOrLast(p->size()), il.instance().goal()[agt]);
  }
}
