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
#include <FloydWarshall.hpp>
#include <Constraint.hpp>

#include "AppConfig.h"
//#include "doctest.h"
#include <Execution.hpp>

#include <doctest/doctest.h>
constexpr Node TEST_SHORTEST_PATH[80][3] = {
    // { target, source, size }
    {362, 968, 22},  {362, 972, 20},  {362, 1406, 63}, {362, 766, 15},  {362, 71, 23},   {362, 1303, 88},
    {362, 1459, 83}, {362, 1486, 55}, {362, 962, 29},  {362, 1349, 66}, {362, 1147, 54}, {362, 643, 15},
    {362, 1155, 35}, {362, 431, 36},  {362, 450, 20},  {362, 1191, 42}, {362, 999, 30},  {362, 472, 17},
    {362, 1370, 86}, {362, 1000, 31}, {360, 987, 55},  {360, 1304, 82}, {360, 1221, 33}, {360, 805, 25},
    {360, 478, 12},  {360, 180, 39},  {360, 1060, 37}, {360, 909, 53},  {360, 88, 29},   {360, 269, 13},
    {360, 1702, 76}, {360, 809, 21},  {360, 1417, 71}, {360, 439, 8},   {360, 647, 16},  {360, 1074, 31},
    {360, 62, 17},   {360, 632, 20},  {360, 871, 49},  {360, 1512, 44}, {402, 1159, 91}, {402, 925, 27},
    {402, 1722, 62}, {402, 1167, 56}, {402, 1199, 84}, {402, 1113, 30}, {402, 1628, 51}, {402, 1471, 67},
    {402, 631, 28},  {402, 641, 15},  {402, 837, 46},  {402, 1311, 80}, {402, 975, 21},  {402, 714, 24},
    {402, 1418, 75}, {402, 1282, 86}, {402, 1708, 77}, {402, 1690, 69}, {402, 886, 26},  {402, 1398, 83},
    {320, 1288, 63}, {320, 846, 26},  {320, 1567, 74}, {320, 109, 17},  {320, 1565, 72}, {320, 139, 31},
    {320, 121, 19},  {320, 751, 25},  {320, 808, 20},  {320, 670, 24},  {320, 278, 4},   {320, 566, 19},
    {320, 251, 13},  {320, 398, 9},   {320, 1431, 69}, {320, 920, 29},  {320, 679, 23},  {320, 1456, 74},
    {320, 1026, 56}, {320, 470, 16}
};

TEST_CASE("Testing of FloydWarshall") {
  instance::XMLInstanceLoader il(std::string(PROJECT_SOURCE_DIR) + "/tests/assets/Test1.exp",
                                 std::string(PROJECT_SOURCE_DIR) + "/tests/assets/");
  il.Load();

  FloydWarshall<ExplicitGraph, ExplicitGraph> fw(il.instance());
  fw.Compute();

  for (size_t i = 0; i < 80; i++) {
    Path p = fw.GetShortestPath(TEST_SHORTEST_PATH[i][1], TEST_SHORTEST_PATH[i][0]);
    CHECK_EQ(p[0], TEST_SHORTEST_PATH[i][1]);
    CHECK_EQ(p[p.size() - 1], TEST_SHORTEST_PATH[i][0]);
    CHECK_EQ(p.size(), TEST_SHORTEST_PATH[i][2]);
  }
}
