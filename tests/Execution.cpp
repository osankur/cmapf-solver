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
#include <Execution.hpp>
#include <sstream>

#include "doctest.h"

TEST_CASE("Testing of Execution class") {
  Execution e;
  CHECK_EQ(e.size(), 0);

  Path p1;
  p1.PushBack(1);
  p1.PushBack(2);
  p1.PushBack(3);
  p1.PushBack(4);
  e.PushBack(std::make_shared<const Path>(p1));
  CHECK_EQ(e.size(), 1);

  Path p2;
  p2.PushBack(9);
  p2.PushBack(8);
  p2.PushBack(7);
  e.PushBack(std::make_shared<const Path>(p2));

  CHECK_EQ(e.size(), 2);
  CHECK_EQ(e.get_path(0)->size(), 4);
  CHECK_EQ(e.get_path(1)->size(), 3);
  CHECK_EQ(e.get_configuration(1).size(), 2);
  CHECK_EQ(e.get_configuration(1)[0], 2);
  CHECK_EQ(e.get_configuration(1)[1], 8);
  CHECK_EQ(e.get_configuration(3).size(), 2);
  CHECK_EQ(e.get_configuration(3)[0], 4);
  CHECK_EQ(e.get_configuration(3)[1], 7);

  Path p3;
  p3.PushBack(15);
  p3.PushBack(16);
  p3.PushBack(17);
  e.set_path(0, std::make_shared<const Path>(p3));

  CHECK_EQ(e.size(), 2);
  CHECK_EQ(e.get_path(0)->size(), 3);
  CHECK_EQ(e.get_path(1)->size(), 3);
  CHECK_EQ(e.get_configuration(3).size(), 2);
  CHECK_EQ(e.get_configuration(3)[0], 17);
  CHECK_EQ(e.get_configuration(3)[1], 7);

  std::stringstream out;
  out << e;
  CHECK_EQ(out.str(), "{[15, 16, 17], [9, 8, 7]}");
}
