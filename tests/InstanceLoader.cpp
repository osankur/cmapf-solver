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

#include "AppConfig.h"
#include "doctest.h"

TEST_CASE("Testing of XMLInstanceLoader class") {
  instance::XMLInstanceLoader il(
      std::string(PROJECT_SOURCE_DIR) + "/tests/assets/Test1.exp",
      std::string(PROJECT_SOURCE_DIR) + "/tests/assets/");
  il.Load();

  CHECK_EQ(il.instance().graph().movement().node_count(), 1750);
  CHECK_EQ(il.instance().graph().movement().edge_count(), 3168 * 2 + 1750);
  CHECK_EQ(il.instance().graph().communication().node_count(), 1750);
  CHECK_EQ(il.instance().graph().communication().edge_count(),
      19494 * 2 + 1750);

  CHECK_EQ(il.instance().nb_agents(), 4);

  CHECK_EQ(il.instance().start().size(), 4);
  CHECK_EQ(il.instance().start()[0], 639);
  CHECK_EQ(il.instance().start()[1], 640);
  CHECK_EQ(il.instance().start()[2], 642);
  CHECK_EQ(il.instance().start()[3], 637);

  CHECK_EQ(il.instance().goal().size(), 4);
  CHECK_EQ(il.instance().goal()[0], 362);
  CHECK_EQ(il.instance().goal()[1], 360);
  CHECK_EQ(il.instance().goal()[2], 402);
  CHECK_EQ(il.instance().goal()[3], 320);
}
