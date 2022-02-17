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
#include <list>
#include <numeric>
#include <map>
#include <functional>
#include <unordered_set>
#include <vector>
#include <memory>
#include <Common.hpp>
#include <Constraint.hpp>
#include <FloydWarshall.hpp>
#include <Instance.hpp>
#include <Path.hpp>
#include <Logger.hpp>
#include <boost/heap/fibonacci_heap.hpp>

namespace decoupled {

template <class GraphMove, class GraphComm>
class LowLevel {
 private:
  std::vector<std::vector<Node>> mem_paths_;

 protected:
  const Instance<GraphMove, GraphComm> &instance_;

  /**
   * Check whether there exists a negative constraint.
   *
   * @param cons Container of the constraints.
   * @param time Timestep of the verification.
   * @param n Node of the verification.
   * @return Whether `cons` contains a negative constraint at time `time` and node `n`.
   */
  bool IsNegativelyConstrained(const std::map<uint64_t, std::list<Constraint>> &cons, uint64_t time, Node n) const {
    auto found = cons.find(time);
    if (found == cons.end()) return false;

    auto &cons_time = found->second;
    for (auto &c : cons_time)
      if (c.type == false && c.node == n) return true;
    return false;
  }

  /**
   * Check whether there is or will be a negative constraint.
   *
   * @param cons Container of the constraints.
   * @param time Timestep of the verification.
   * @param n Node of the verification.
   * @return Whether `cons` contains a negative constraint at time `time` or later and node `n`.
   */
  bool IsLaterNegativelyConstrained(const std::map<uint64_t, std::list<Constraint>> &cons, uint64_t time,
                                    Node n) const {
    for (auto &lc : cons)
      if (lc.first >= time)
        for (auto &c : lc.second)
          if (c.type == false && c.node == n) return true;
    return false;
  }

  /**
   * Check whether there exists a positive constraint.
   *
   * @param cons Container of the constraints.
   * @param time Timestep of the verification.
   * @param n Node of the verification.
   * @return Whether `cons` contains a positive constraint at time `time` and away from node `n`.
   */
  bool IsPositivelyConstrained(const std::map<uint64_t, std::list<Constraint>> &cons, uint64_t time, Node n) const {
    const auto found = cons.find(time);
    if (found == cons.end()) return false;

    const auto &cons_time = found->second;
    for (const auto &c : cons_time)
      if (c.type == true && c.node != n) return true;
    return false;
  }

  /**
   * Check whether there is or will be a negative constraint.
   *
   * @param cons Container of the constraints.
   * @param time Timestep of the verification.
   * @param n Node of the verification.
   * @return Whether `cons` contains a positive constraint at time `time` or later and away from node `n`.
   */
  bool IsLaterPositivelyConstrained(const std::map<uint64_t, std::list<Constraint>> &cons, uint64_t time,
                                    Node n) const {
    for (auto &lc : cons)
      if (lc.first >= time)
        for (auto &c : lc.second)
          if (c.type == true && c.node != n) return true;
    return false;
  }

 public:
  explicit LowLevel(const Instance<GraphMove, GraphComm> &instance) : instance_(instance), mem_paths_() {}
  virtual ~LowLevel() {}

  virtual Path ComputeShiftedConstrainedPath(const std::map<uint64_t, std::list<Constraint>> &, const Constraint &,
                                             const Node &, const Node &, uint64_t) = 0;
  Path ComputeConstrainedPath(const std::map<uint64_t, std::list<Constraint>> &cons, const Constraint &c,
                              const Node &source, const Node &target) {
    return ComputeShiftedConstrainedPath(cons, c, source, target, 0);
  }
  Path ComputeShortestPath(const Node &source, const Node &target) {
    return ComputeConstrainedPath(std::map<uint64_t, std::list<Constraint>>(), Constraint{0, 0, false}, source, target);
  }
};

namespace low_level {

template <class GraphMove, class GraphComm>
class NegativeAStar : public LowLevel<GraphMove, GraphComm> {
 private:
  struct AStarNode : std::enable_shared_from_this<AStarNode> {
   public:
    const Node node;
    const uint64_t time;
    std::shared_ptr<AStarNode> pred;

    AStarNode(Node n, uint64_t t, std::shared_ptr<AStarNode> p) : node(n), time(t), pred(p) {}
  };

  struct AStarNodePtrEqual {
    bool operator()(const std::shared_ptr<AStarNode> &a, const std::shared_ptr<AStarNode> &b) const {
      return a->node == b->node && a->time && b->time;
    }
  };

  struct AStarNodePtrHash {
    size_t operator()(const std::shared_ptr<AStarNode> &that) const {
      return std::hash<Node>()(that->node) ^ std::hash<uint64_t>()(that->time);
    }
  };

  class HeapComparator {
   private:
    const Instance<GraphMove, GraphComm> &instance_;
    const Node &target_;

   public:
    explicit HeapComparator(const Instance<GraphMove, GraphComm> &instance, const Node &target)
        : instance_(instance), target_(target) {}
    bool operator()(const std::shared_ptr<AStarNode> &a, const std::shared_ptr<AStarNode> &b) const {
      return a->time + instance_.graph().movement().get_distance(a->node, target_) >
             b->time + instance_.graph().movement().get_distance(b->node, target_);
    }
  };

  // TODO(arqueffe): Avoid reversing process
  Path RetrievePath(std::shared_ptr<AStarNode> end) const {
    std::vector<std::shared_ptr<AStarNode>> reverse_path;
    std::shared_ptr<AStarNode> current = end;
    Path p;
    while (current->pred != nullptr) {
      reverse_path.push_back(current);
      current = current->pred;
    }
    reverse_path.push_back(current);
    for (auto r_it = reverse_path.rbegin(); r_it != reverse_path.rend(); ++r_it) p.PushBack((*r_it)->node);
    return p;
  }

 public:
  explicit NegativeAStar(const Instance<GraphMove, GraphComm> &instance) : LowLevel<GraphMove, GraphComm>(instance) {}

  Path ComputeShiftedConstrainedPath(const std::map<uint64_t, std::list<Constraint>> &cons, const Constraint &c,
                                     const Node &source, const Node &target, uint64_t time) override {
    HeapComparator cmp(this->instance_, target);
    boost::heap::fibonacci_heap<std::shared_ptr<AStarNode>, boost::heap::compare<HeapComparator>> open(cmp);
    std::unordered_set<std::shared_ptr<AStarNode>, AStarNodePtrHash, AStarNodePtrEqual> closed;

    std::shared_ptr<AStarNode> start = std::make_shared<AStarNode>(source, time, nullptr);
    open.push(start);
    closed.insert(start);
    while (!open.empty()) {
      std::shared_ptr<AStarNode> current = open.top();
      open.pop();

      if (current->node == target && !this->IsLaterNegativelyConstrained(cons, current->time, current->node) &&
          !(c.type == false && c.time >= current->time && c.node == current->node))
        return RetrievePath(current);

      for (Node neighbor : this->instance_.graph().movement().get_neighbors(current->node)) {
        if (this->IsNegativelyConstrained(cons, current->time + 1, neighbor) ||
            (c.type == false && current->time + 1 == c.time && neighbor == c.node)) {
          continue;
        }

        std::shared_ptr<AStarNode> next = std::make_shared<AStarNode>(neighbor, current->time + 1, current);

        if (closed.find(next) == closed.end()) {
          open.push(next);
          closed.insert(next);
        }
      }
    }
    return Path();
  }
};

template <class GraphMove, class GraphComm>
class PositiveAStar : public LowLevel<GraphMove, GraphComm> {
 private:
  FloydWarshall<GraphMove, GraphComm> fw_;

 public:
  explicit PositiveAStar(const Instance<GraphMove, GraphComm> &instance)
      : LowLevel<GraphMove, GraphComm>(instance), fw_(instance) {}

  Path ComputeShiftedConstrainedPath(const std::map<uint64_t, std::list<Constraint>> &cons, const Constraint &c,
                                     const Node &source, const Node &target, uint64_t time) {
    if (time > 0) LOG_WARNING("Shifted computation may be usafe!");

    std::map<uint64_t, Constraint> constraints;
    if (c.time > 0 && c.type == true) constraints.insert(std::make_pair(c.time - time, c));
    for (const auto &time_cons : cons) {
      for (Constraint constraint : time_cons.second) {
        if (constraint.type == true) {
          constraints.insert(std::make_pair(constraint.time - time, constraint));
          break;
        }
      }
    }
#ifdef DEBUG
    size_t test_size = cons.size();
    test_size += c.time > 0 ? 1 : 0;
    if (constraints.size() != test_size) throw "Size mismatch";
#endif
    if (constraints.empty()) return fw_.getShortestPath(source, target);

    Path final_path;
    auto iter = constraints.cbegin();
    // Source to first constraint
    Path p = fw_.getShortestPath(source, iter->second.node);
    if (p.size() == 0 || p.size() - 1 > (iter->second.time - time)) return Path();
    for (size_t i = 0; i < p.size(); i++) final_path.PushBack(p.at(i));
    while (final_path.size() <= (iter->second.time - time)) final_path.PushBack(final_path.at(final_path.size() - 1));

    // Constraint to Constraint
    while (std::next(iter) != constraints.cend()) {
      p = fw_.getShortestPath(iter->second.node, std::next(iter)->second.node);
      if (p.size() == 0 || p.size() - 1> (std::next(iter)->second.time - iter->second.time)) return Path();
      for (size_t i = 1; i < p.size(); i++) final_path.PushBack(p.at(i));
      while (final_path.size() <= (std::next(iter)->second.time - time))
        final_path.PushBack(final_path.at(final_path.size() - 1));
      iter++;
    }
    // Constraint to target
    if (final_path.at(final_path.size() - 1) != target) {
      p = fw_.getShortestPath(constraints.crbegin()->second.node, target);
      if (p.size() == 0) return Path();
      for (size_t i = 1; i < p.size(); i++) final_path.PushBack(p.at(i));
    }
#ifdef DEBUG
    for (auto &cns : constraints)
      if (final_path[cns.second.time] != cns.second.node) throw "Error path";
#endif
    return final_path;
    /*HeapComparator cmp(this->instance_, target, fw_);
    boost::heap::fibonacci_heap<std::shared_ptr<AStarNode>, boost::heap::compare<HeapComparator>> open(cmp);
    std::unordered_set<std::shared_ptr<AStarNode>, AStarNodePtrHash, AStarNodePtrEqual> closed;

    std::shared_ptr<AStarNode> start = std::make_shared<AStarNode>(source, time, nullptr);
    open.push(start);
    closed.insert(start);
    while (!open.empty()) {
      std::shared_ptr<AStarNode> current = open.top();
      open.pop();

      if (current->node == target && !this->IsLaterPositivelyConstrained(cons, current->time, current->node) &&
          !(c.type == true && c.time >= current->time && c.node != current->node))
        return RetrievePath(current);

      const auto &neighbors = this->instance_.graph().movement().get_neighbors(current->node);

      if (c.type == true && current->time + 1 == c.time) {
        if (neighbors.find(c.node) != neighbors.end()) {
          std::shared_ptr<AStarNode> next = std::make_shared<AStarNode>(c.node, current->time + 1, current);

          if (closed.find(next) == closed.end()) {
            open.push(next);
            closed.insert(next);
          }
        }
        continue;
      }

      const auto &nextCons = cons.find(current->time + 1);
      if (nextCons != cons.end()) {
        const auto &cons_time = nextCons->second;
        for (const auto &constr : cons_time) {
          if (constr.type == true) {
            if (neighbors.find(constr.node) != neighbors.end()) {
              std::shared_ptr<AStarNode> next = std::make_shared<AStarNode>(constr.node, current->time + 1, current);

              if (closed.find(next) == closed.end()) {
                open.push(next);
                closed.insert(next);
              }
            }
            continue;
          }
        }
      }

      for (Node neighbor : neighbors) {
        std::shared_ptr<AStarNode> next = std::make_shared<AStarNode>(neighbor, current->time + 1, current);

        if (closed.find(next) == closed.end()) {
          open.push(next);
          closed.insert(next);
        }
      }
    }
    return Path();*/
  }
};

}  // namespace low_level

}  // namespace decoupled
