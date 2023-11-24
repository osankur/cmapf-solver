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
#include <CCBS.hpp>
#include <InstanceLoader.hpp>
#include <Objective.hpp>
#include "CommonTest.hpp"

#include "AppConfig.h"
#include <doctest/doctest.h>

TEST_CASE("Testing of CCBS class") {
  instance::XMLInstanceLoader il(std::string(PROJECT_SOURCE_DIR) + "/tests/assets/Test1.exp",
                                 std::string(PROJECT_SOURCE_DIR) + "/tests/assets/");
  il.Load();

  SumObjective obj;
  decoupled::conflict_selection::FirstConflictStrategy con;

  decoupled::high_level::CCBS<ExplicitGraph, decoupled::ctn_ordering::LeastConflictStrategy> ccbs(il.instance(), obj,
                                                                                                  con);

  /*Execution e = ccbs.Compute();

  for (Agent agt = 0; agt < static_cast<Agent>(il.instance().nb_agents()); agt++) {
    auto p = e.get_path(agt);
    CHECK_EQ(p->GetAtTimeOrLast(0), il.instance().start()[agt]);
    CHECK_EQ(p->GetAtTimeOrLast(p->size()), il.instance().goal()[agt]);
  }
  CHECK(IsExecutionConnected(e, il.instance()));*/
  /*MaxObjective max;
  size_t len = max.cost(e);
  for (size_t time = 0; time < len; time++) {
    for (Agent agtA = 0; agtA < static_cast<Agent>(il.instance().nb_agents()); agtA++) {
      for (Agent agtB = agtA + 1; agtB < static_cast<Agent>(il.instance().nb_agents()); agtB++) {
        auto config = e.get_configuration(time);
        CHECK_NE(config[agtA], config[agtB]);
      }
    }
  }

  std::cout << "CCBS :" << obj.cost(e) << std::endl;
  */
}
