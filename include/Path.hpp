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
#include <iostream>
#include <vector>
#include <Common.hpp>

class Path {
 private:
  std::vector<Node> path_;

 public:
  void PushBack(Node);
  const Node& GetAtTimeOrLast(size_t) const;

  // Operators
  Node& operator[](size_t);
  const Node& operator[](size_t) const;

  // Properties
  size_t size() const;

  // Friends
  friend std::ostream& operator<<(std::ostream&, const Path&);
};
