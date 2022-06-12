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

#include <CMAPF.hpp>
#include <Graph.hpp>

template <class GraphMove, class GraphComm>
class TopologicalGraph {
 private:
  GraphMove movement_graph_;
  GraphComm communication_graph_;

 public:
  TopologicalGraph() = default;
  TopologicalGraph(const TopologicalGraph&) = default;
  GraphMove& movement() { return movement_graph_; }

  GraphComm& communication() { return communication_graph_; }

  const GraphMove& movement() const { return movement_graph_; }

  const GraphComm& communication() const { return communication_graph_; }
};
