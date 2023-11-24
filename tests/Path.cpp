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
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <Path.hpp>
#include <sstream>

//#include "doctest.h"

#include <doctest/doctest.h>
TEST_CASE("Testing of Path class") {
  Path p;
  CHECK_EQ(p.size(), 0);
  p.PushBack(1);
  CHECK_EQ(p.size(), 1);
  p.PushBack(2);
  CHECK_EQ(p.size(), 2);
  CHECK_EQ(p[0], 1);
  CHECK_EQ(p[1], 2);
  p[0] = 3;
  CHECK_EQ(p[0], 3);

  const Path p2(p);
  CHECK_EQ(p2.size(), 2);
  CHECK_EQ(p2[1], 2);
  CHECK_EQ(p2[0], 3);

  std::stringstream out;
  out << p;
  CHECK_EQ(out.str(), "[3, 2]");
  p.PushBack(6);
  out.str(std::string());
  out.clear();
  out << p;
  CHECK_EQ(out.str(), "[3, 2, 6]");
}
