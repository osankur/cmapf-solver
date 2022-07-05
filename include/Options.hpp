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
#pragma once
#include <stdint.h>
#include <cmath>

enum class SubsolverEnum : int { DECOUPLED_SOLVER, DFS_SOLVER, COORD_SOLVER };
enum class Algorithm : int
{
  CBS = 0,
  CCBS,
  CASTAR,
  DFS,
  COORD,
  CMARRT,
  CMARRTSTAR
};
enum class ObjectiveEnum : int
{
  SUM = 0,
  MAX
};

enum class HeuristicsEnum : int
{
  SHORTEST_PATH,
  BIRDEYE
};
enum class CollisionMode : int {CHECK_COLLISIONS, IGNORE_COLLISIONS};


#define ANSI_BOLD  "\u001B[1m"
#define ANSI_RESET  "\u001B[0m"
#define ANSI_BLACK  "\u001B[30m"
#define ANSI_RED  "\u001B[31m"
#define ANSI_GREEN  "\u001B[32m"
#define ANSI_YELLOW  "\u001B[33m"
#define ANSI_BLUE  "\u001B[34m"
#define ANSI_PURPLE  "\u001B[35m"
#define ANSI_CYAN  "\u001B[36m"
#define ANSI_WHITE  "\u001B[37m"

