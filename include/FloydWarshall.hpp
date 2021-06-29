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

 protected:
  const Instance<GraphMove, GraphComm> &instance_;

 public:
  explicit FloydWarshall(const Instance<GraphMove, GraphComm> &instance) : instance_(instance), mem_paths_() {}
  virtual ~FloydWarshall() {}

  Path GetShortestPath(const Node &source, const Node &target) const {
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

  size_t GetShortestPathSize(const Node &source, const Node &target) const {
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

  /*void StoreShortestPath(const Node &target, const Path &path) {
    if (target >= mem_paths_.size()) mem_paths_.resize(target + 1);
    for (size_t step = 0; step < path.size() - 1; step++) {
      if (path[step] >= mem_paths_[target].size())
        mem_paths_[target].resize(path[step] + 1, std::numeric_limits<Node>::max());
      mem_paths_[target][path[step]] = path[step + 1];
    }
  }*/

  void Compute() {
    const Configuration &goal = instance_.goal();

    for (size_t i = 0; i < goal.size(); i++) {
      Node target = goal[i];

      mem_paths_.insert(std::make_pair(target, std::vector<Node>{}));
      std::vector<Node> &mem = mem_paths_[target];
      mem.resize(instance_.graph().movement().node_count(), std::numeric_limits<Node>::max());
      std::list<Node> open;
      std::unordered_set<Node> closed;

      mem[target] = target;
      open.push_back(target);

      while (!open.empty()) {
        Node head = open.front();
        open.pop_front();
        closed.insert(head);

        for (Node neighbor : instance_.graph().movement().get_neighbors(head)) {
          if (closed.find(neighbor) != closed.end()) continue;
          if (GetShortestPathSize(head, target) + 1 < GetShortestPathSize(neighbor, target)) mem[neighbor] = head;
          if (find(open.begin(), open.end(), neighbor) == open.end()) open.push_back(neighbor);
        }
      }
    }
  }
};
