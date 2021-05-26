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

#include <list>
#include <set>
#include <Common.hpp>

namespace decoupled {

class Conflict {
 public:
  virtual size_t size() const = 0;
};

class CollisionConflict : public Conflict {
 private:
  std::list<std::set<Agent>> clusters_;

 public:
  size_t size() const override { return clusters_.size(); }
  void PushBack(const std::set<Agent>& cluster) { clusters_.push_back(cluster); }
  const std::set<Agent>& back() const { return clusters_.back(); }
  std::set<Agent>& back() { return clusters_.back(); }
};

class DisconnectionConflict : public Conflict {
 private:
  std::list<std::set<Agent>> clusters_;

 public:
  size_t size() const override { return clusters_.size() - 1; }
  void PushBack(const std::set<Agent>& cluster) { clusters_.push_back(cluster); }
  const std::set<Agent>& back() const { return clusters_.back(); }
  std::set<Agent>& back() { return clusters_.back(); }
};

}  // namespace decoupled
