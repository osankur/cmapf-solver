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
#include <Graph.hpp>

//#include "doctest.h"

#include <doctest/doctest.h>
TEST_CASE("Testing of ExplicitGraph class") {
  ExplicitGraph eg;
  CHECK_EQ(eg.node_count(), 0);
  eg.AddNode(0);
  CHECK_EQ(eg.node_count(), 1);
  eg.AddNode(1);
  CHECK_EQ(eg.node_count(), 2);
  eg.AddEdge(0, 1);
  CHECK_EQ(eg.node_count(), 2);
  CHECK_EQ(eg.edge_count(), 1);
  eg.AddEdge(1, 0);
  CHECK_EQ(eg.node_count(), 2);
  CHECK_EQ(eg.edge_count(), 2);
  eg.AddEdge(1, 0);
  CHECK_EQ(eg.node_count(), 2);
  CHECK_EQ(eg.edge_count(), 2);
  eg.AddEdge(1, 1);
  CHECK_EQ(eg.node_count(), 2);
  CHECK_EQ(eg.edge_count(), 3);
  eg.AddEdge(0, 0);
  CHECK_EQ(eg.node_count(), 2);
  CHECK_EQ(eg.edge_count(), 4);
  CHECK_EQ(eg.get_neighbors(0).size(), 2);
  CHECK_EQ(eg.get_neighbors(1).size(), 2);

  eg.AddPosition(0, 0, 0);
  eg.AddPosition(1, 0, 1);
  CHECK_EQ(eg.get_distance(0, 0), 0);
  CHECK_EQ(eg.get_distance(1, 1), 0);
  CHECK_EQ(eg.get_distance(1, 0), 1);
  CHECK_EQ(eg.get_distance(0, 1), 1);
}
