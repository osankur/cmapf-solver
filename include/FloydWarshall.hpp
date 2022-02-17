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

#include <limits>
#include <utility>
#include <list>
#include <numeric>
#include <map>
#include <functional>
#include <unordered_set>
#include <vector>
#include <memory>
#include <Common.hpp>
#include <Constraint.hpp>
#include <Instance.hpp>
#include <Path.hpp>
#include <boost/heap/fibonacci_heap.hpp>

template <class GraphMove, class GraphComm>
class FloydWarshall {
 private:
  std::map<Node, std::vector<Node>> mem_paths_;

  enum class NodeColor { kWhite, kGray, kBlack };

  void computeShortestPaths(const Node &target) {
    mem_paths_.insert(std::make_pair(target, std::vector<Node>{}));
    std::vector<Node> &mem = mem_paths_[target];
    mem.resize(instance_.graph().movement().node_count(), std::numeric_limits<Node>::max());
    std::list<Node> open;
    std::vector<NodeColor> colored(instance_.graph().movement().node_count(), NodeColor::kWhite);

    mem[target] = target;
    open.push_back(target);

    while (!open.empty()) {
      Node head = open.front();
      open.pop_front();
      colored[head] = NodeColor::kBlack;

      for (Node neighbor : instance_.graph().movement().get_neighbors(head)) {
        if (colored[neighbor] == NodeColor::kBlack) continue;
        if (getShortestPathDistance(head, target) + 1 < getShortestPathDistance(neighbor, target)) mem[neighbor] = head;
        if (colored[neighbor] == NodeColor::kWhite) {
          open.push_back(neighbor);
          colored[neighbor] = NodeColor::kGray;
        }
      }
    }
  }

 protected:
  const Instance<GraphMove, GraphComm> &instance_;

 public:
  explicit FloydWarshall(const Instance<GraphMove, GraphComm> &instance) : mem_paths_(), instance_(instance) {}
  virtual ~FloydWarshall() {}

  Path getShortestPath(const Node &source, const Node &target) {
    auto memIt = mem_paths_.find(target);
    if (memIt == mem_paths_.end()) computeShortestPaths(target);
    const std::vector<Node> &mem = mem_paths_.at(target);
    Path p;
    Node current = source;
    while (current != target) {
      p.PushBack(current);
      if (mem.at(current) == current || mem.at(current) == std::numeric_limits<Node>::max()) throw "Unconnected Graph!";
      current = mem.at(current);
    }
    p.PushBack(current);
    return p;
  }

  size_t getShortestPathDistance(const Node &source, const Node &target) {
    auto memIt = mem_paths_.find(target);
    if (memIt == mem_paths_.end()) computeShortestPaths(target);
    const std::vector<Node> &mem = mem_paths_.at(target);
    size_t size = 0;
    Node current = source;
    while (current != target) {
      size++;
      if (mem.at(current) == std::numeric_limits<Node>::max()) return std::numeric_limits<Node>::max();
      if (mem.at(current) == current) throw "Unconnected Graph!";
      current = mem.at(current);
    }
    size++;
    return size;
  }

  void computeAllPairs() {
    const Configuration &goal = instance_.goal();

    for (size_t i = 0; i < goal.size(); i++) {
      Node target = goal[i];
      computeShortestPaths(target);
    }
  }
};
