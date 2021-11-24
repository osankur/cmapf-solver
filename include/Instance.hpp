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
#include <memory>
#include <Configuration.hpp>
#include <TopologicalGraph.hpp>

template <class GraphMove, class GraphComm>
class Instance {
 private:
  TopologicalGraph<GraphMove, GraphComm> topo_graph_;
  Configuration start_;
  Configuration goal_;
  size_t nb_agents_;

 public:
  Instance() : topo_graph_(), start_(), goal_(), nb_agents_(0) {}
  Instance(const TopologicalGraph<GraphMove,GraphComm> & topo_graph,
           const Configuration & start,
           const Configuration & goal) : topo_graph_(topo_graph), start_(start), goal_(goal), nb_agents_(start.size())
           {}
  ~Instance() = default;
  Instance(const Instance& other) = default;
  Instance(Instance&& other) = default;
  Instance& operator=(const Instance& other) = default;
  Instance& operator=(Instance&& other) = default;

  Configuration& start() { return start_; }
  Configuration& goal() { return goal_; }
  void set_nb_agents(size_t nb) { nb_agents_ = nb; }
  void set_start(const Configuration newstart) { start_ = newstart;}
  void set_goal(const Configuration newgoal) { goal_ = newgoal;}

  const TopologicalGraph<GraphMove, GraphComm>& graph() const { return topo_graph_; }
  TopologicalGraph<GraphMove, GraphComm>& graph() { return topo_graph_; }
  const Configuration& start() const { return start_; }
  const Configuration& goal() const { return goal_; }
  size_t nb_agents() const { return nb_agents_; }
};
