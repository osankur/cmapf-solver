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
#include <Configuration.hpp>
#include <sstream>

#include "doctest.h"

TEST_CASE("Testing of Configuration class") {
  Configuration c;
  CHECK_EQ(c.size(), 0);
  c.PushBack(1);
  CHECK_EQ(c.size(), 1);
  c.PushBack(2);
  CHECK_EQ(c.size(), 2);
  CHECK_EQ(c[0], 1);
  CHECK_EQ(c[1], 2);
  c[0] = 3;
  CHECK_EQ(c[0], 3);

  const Configuration c2(c);
  CHECK_EQ(c2.size(), 2);
  CHECK_EQ(c2[1], 2);
  CHECK_EQ(c2[0], 3);

  std::stringstream out;
  out << c;
  CHECK_EQ(out.str(), "<3, 2>");
  c.PushBack(6);
  out.str(std::string());
  out.clear();
  out << c;
  CHECK_EQ(out.str(), "<3, 2, 6>");
}
