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

constexpr Node TEST_SHORTEST_PATH[50][3] = {  // { source, target, size }
    {1265, 1014, 78}, {1685, 1188, 45}, {655, 1629, 57},  {616, 420, 12},   {1739, 812, 69},  {1729, 717, 49},
    {1523, 1337, 9},  {1033, 720, 18},  {1247, 948, 32},  {720, 84, 37},    {1537, 1634, 40}, {507, 1352, 78},
    {312, 373, 23},   {655, 199, 21},   {699, 1552, 72},  {1711, 351, 80},  {506, 64, 16},    {1002, 804, 8},
    {1440, 1217, 11}, {832, 1693, 52},  {1485, 1096, 53}, {1345, 1029, 37}, {257, 735, 23},   {270, 217, 3},
    {888, 191, 24},   {1169, 1567, 47}, {1008, 933, 3},   {792, 1011, 54},  {217, 150, 49},   {420, 118, 31},
    {552, 674, 41},   {1355, 76, 65},   {505, 800, 12},   {1143, 1306, 48}, {332, 1172, 47},  {961, 947, 73},
    {601, 763, 14},   {685, 32, 28},    {290, 262, 30},   {1105, 1161, 75}, {1152, 35, 53},   {556, 1590, 73},
    {140, 1383, 76},  {897, 1605, 59},  {1262, 598, 77},  {585, 573, 32},   {302, 1669, 44},  {674, 411, 36},
    {163, 1326, 55},  {1565, 195, 82}};

TEST_CASE("Testing of NegativeAStar::ComputeShortestPath") {
  instance::XMLInstanceLoader il(std::string(PROJECT_SOURCE_DIR) + "/tests/assets/Test1.exp",
                                 std::string(PROJECT_SOURCE_DIR) + "/tests/assets/");
  il.Load();

  decoupled::low_level::NegativeAStar<ExplicitGraph, ExplicitGraph> astar(il.instance());

  for (size_t i = 0; i < 50; i++) {
    Path p = astar.ComputeShortestPath(TEST_SHORTEST_PATH[i][0], TEST_SHORTEST_PATH[i][1]);
    CHECK_EQ(p[0], TEST_SHORTEST_PATH[i][0]);
    CHECK_EQ(p[p.size() - 1], TEST_SHORTEST_PATH[i][1]);
    CHECK_EQ(p.size(), TEST_SHORTEST_PATH[i][2]);
  }
}

TEST_CASE("Testing of PositiveAStar::ComputeShortestPath") {
  instance::XMLInstanceLoader il(std::string(PROJECT_SOURCE_DIR) + "/tests/assets/Test1.exp",
                                 std::string(PROJECT_SOURCE_DIR) + "/tests/assets/");
  il.Load();

  decoupled::low_level::PositiveAStar<ExplicitGraph, ExplicitGraph> astar(il.instance());

  for (size_t i = 0; i < 50; i++) {
    Path p = astar.ComputeShortestPath(TEST_SHORTEST_PATH[i][0], TEST_SHORTEST_PATH[i][1]);
    CHECK_EQ(p[0], TEST_SHORTEST_PATH[i][0]);
    CHECK_EQ(p[p.size() - 1], TEST_SHORTEST_PATH[i][1]);
    CHECK_EQ(p.size(), TEST_SHORTEST_PATH[i][2]);
  }
}
