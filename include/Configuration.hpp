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

#include <ostream>
#include <vector>
#include <Common.hpp>

class Configuration {
 private:
  std::vector<Node> config_;

 public:
  void PushBack(Node);

  // Operators
  Node& operator[](Agent);
  const Node& operator[](Agent) const;

  // Properties
  size_t size() const;
  const Node& at(Agent) const;

  // Friends
  friend std::ostream& operator<<(std::ostream&, const Configuration&);
};
