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

#include <Execution.hpp>

class Objective {
 public:
  virtual ~Objective() {}
  virtual size_t cost(const Execution&) const = 0;
};

class MaxObjective : public Objective {
 public:
  size_t cost(const Execution& e) const override {
    size_t max = 0;
    for (Agent i = 0; i < static_cast<Agent>(e.size()); i++) {
      size_t size = e.getPath(i)->size();
      if (max < size) max = size;
    }
    return max;
  }
};

class SumObjective : public Objective {
 public:
  size_t cost(const Execution& e) const override {
    size_t sum = 0;
    for (Agent i = 0; i < static_cast<Agent>(e.size()); i++)
      if (e.getPath(i) != nullptr) sum += e.getPath(i)->size();
    return sum;
  }
};
