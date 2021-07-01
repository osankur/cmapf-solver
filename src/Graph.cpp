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

size_t ExplicitGraph::node_count() const { return adjacency_.size(); }

size_t ExplicitGraph::edge_count() const {
  size_t sum = 0;
  for (auto& edges : adjacency_) sum += edges.size();
  return sum;
}

void ExplicitGraph::AddNode(Node n) {
  if (adjacency_.size() <= n) adjacency_.resize(n);
  adjacency_.insert(adjacency_.begin() + n, std::unordered_set<Node>());
}

void ExplicitGraph::AddEdge(Node source, Node target) { adjacency_[source].insert(target); }

void ExplicitGraph::AddPosition(Node n, int x, int y) {
  if (positions_.size() <= n) positions_.resize(n);
  positions_.insert(positions_.begin() + n, std::make_pair(x, y));
}

const std::unordered_set<Node>& ExplicitGraph::get_neighbors(Node n) const { return adjacency_[n]; }

size_t ExplicitGraph::get_distance(Node a, Node b) const {
  return static_cast<size_t>(abs(positions_[a].first - positions_[b].first) +
                             abs(positions_[a].second - positions_[b].second));
}
