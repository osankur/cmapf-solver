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
#include <Instance.hpp>
#include <Objective.hpp>

class Solver {
 private:
  const instance::Instance& instance_;
  const Objective& objective_;
  Execution execution_;

 public:
  Solver(const Instance& instance, const Objective& objective)
      : instance_(instance), objective_(objective), execution_() {}

  virtual bool StepCompute() = 0;

  const Execution Compute() {
    while (!StepCompute()) {}
    return execution_;
  }

  const instance::Instance& instance() const { return instance_; }
  const Objective& objective() const { return objective_; }
};
