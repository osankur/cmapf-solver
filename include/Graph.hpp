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

#include <vector>
#include <tuple>
#include <utility>
#include <Common.hpp>
#include <unordered_set>

class Graph {
 public:
  virtual const std::unordered_set<Node>& get_neighbors(Node) const = 0;
  virtual size_t node_count() const = 0;
  virtual size_t get_distance(Node, Node) const = 0;
};

class ExplicitGraph : public Graph {
 private:
  std::vector<std::unordered_set<Node>> adjacency_;
  std::vector<std::pair<int, int>> positions_;

 public:
  void AddNode(Node);
  void AddEdge(Node, Node);
  void AddPosition(Node, int x, int y);

  size_t node_count() const override;
  size_t edge_count() const;
  const std::unordered_set<Node>& get_neighbors(Node) const override;
  size_t get_distance(Node, Node) const override;
};

class RadiusGraph : public Graph {
 public:
  const std::unordered_set<Node>& get_neighbors(Node) const override;
};

class GridGraph : public Graph {
 public:
  const std::unordered_set<Node>& get_neighbors(Node) const override;
};
