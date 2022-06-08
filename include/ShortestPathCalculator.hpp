/* 
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
#include <queue>

template <class GraphMove, class GraphComm>
class ShortestPathCalculator {
protected:
    const Instance<GraphMove, GraphComm> &instance_;
    const Instance<GraphMove, GraphComm> & instance() const {
        return instance_;
    }
public:
  explicit ShortestPathCalculator(const Instance<GraphMove, GraphComm> &instance) : instance_(instance) {}
  virtual ~ShortestPathCalculator() {}
  virtual Path  getShortestPath(const Node &source, const Node &target) = 0;
  virtual size_t getShortestPathDistance(const Node &source, const Node &target) = 0;
};


template <class GraphMove, class GraphComm>
class DijkstraSPCalculator : public ShortestPathCalculator<GraphMove,GraphComm>{
 private:
  std::map<Node, std::vector<Node>> mem_paths_;

  enum class NodeColor { kWhite, kGray, kBlack };

  void computeShortestPaths(const Node &target) {
    // std::cout << "Computing shortest paths for target = " << target << "\n";
    mem_paths_.insert(std::make_pair(target, std::vector<Node>{}));
    std::vector<Node> &mem = mem_paths_[target];
    mem.resize(this->instance_.graph().movement().node_count(), std::numeric_limits<Node>::max());
    std::list<Node> open;
    std::vector<NodeColor> colored(this->instance_.graph().movement().node_count(), NodeColor::kWhite);

    mem[target] = target;
    open.push_back(target);

    while (!open.empty()) {
      Node head = open.front();
      open.pop_front();
      colored[head] = NodeColor::kBlack;

      for (Node neighbor : this->instance_.graph().movement().get_neighbors(head)) {
        if (colored[neighbor] == NodeColor::kBlack) continue;
        if (getShortestPathDistance(head, target) + 1 < getShortestPathDistance(neighbor, target)) mem[neighbor] = head;
        if (colored[neighbor] == NodeColor::kWhite) {
          open.push_back(neighbor);
          colored[neighbor] = NodeColor::kGray;
        }
      }
    }
  }

 public:
  DijkstraSPCalculator(const Instance<GraphMove, GraphComm> &instance) : ShortestPathCalculator<GraphMove,GraphComm>(instance), mem_paths_(){}
  virtual ~DijkstraSPCalculator() {}


  Path getShortestPath(const Node &source, const Node &target) override {
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

  size_t getShortestPathDistance(const Node &source, const Node &target) override {
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
};

template <class GraphMove, class GraphComm>
class AStarSPCalculator : public ShortestPathCalculator<GraphMove,GraphComm> {
private:
  DijkstraSPCalculator<GraphMove,GraphComm> dijkstra_;
  std::vector<std::unordered_map<Node, size_t>> dist_;
  std::set<Node> target_nodes_;
protected:

  class AStarNode{
    public:
    AStarNode(Node node, size_t g) : node(node), g(g){}
    Node node;
    size_t g;
  };
  class HeapComparator {
   private:
    const Instance<GraphMove, GraphComm>& instance_;
    const Node& target_;
   public:
    explicit HeapComparator(const Instance<GraphMove, GraphComm>& instance, const Node& target)
        : instance_(instance), target_(target) {}

    double getL1Distance(std::pair<int, int> &p1, std::pair<int, int> &p2) const
    {
        return abs(p1.first - p2.first) + abs(p1.second - p2.second);
    }
    double getL2Distance(std::pair<int, int> &p1, std::pair<int, int> &p2) const
    {
        return sqrt((p1.first - p2.first) * (p1.first - p2.first)+ (p1.second - p2.second) * (p1.second - p2.second));
    }

    bool operator()(AStarNode & a, AStarNode & b) 
     {
      auto apos = this->instance_.graph().movement().getPosition(a.node);
      auto bpos = this->instance_.graph().movement().getPosition(b.node);
      auto tpos = this->instance_.graph().movement().getPosition(target_);

      size_t sizeA = a.g + int(getL2Distance(apos, tpos));
      size_t sizeB = b.g + int(getL2Distance(bpos, tpos));
      return sizeA > sizeB;
    }
  };


public:
  explicit AStarSPCalculator(const Instance<GraphMove, GraphComm> &instance) : ShortestPathCalculator<GraphMove,GraphComm>(instance),
  dijkstra_(DijkstraSPCalculator<GraphMove,GraphComm>(instance))
      {
        dist_.resize(instance.graph().movement().node_count());
        for(auto n : instance.goal()){
          target_nodes_.insert(n);
        }
      }
  virtual ~AStarSPCalculator() {}
  Path getShortestPath(const Node &source, const Node &target) override {
    HeapComparator cmp(this->instance_, target);
    std::priority_queue<AStarNode, std::vector<AStarNode>, HeapComparator> open(cmp);

    std::unordered_map<Node,Node> parent;
    std::unordered_map<Node,size_t> dist;
    AStarNode start(source, 0);
    open.push(AStarNode(start.node, 0));

    parent[source] = source;
    dist[source] = 0;
    // std::cout << "ASTAR: source=" << source << ", target=" << target << "\n";
    int iterations_count = 0;
    while (open.size() > 0){
      iterations_count++;
      auto astar_node=  open.top();
      open.pop();
      // Invariant: astar_node.g is the shortest path distance from source to astar_node.node
      dist_[source][astar_node.node] = astar_node.g;
      if (astar_node.node == target){
        break;
      }
      auto neighbors = this->instance_.graph().movement().get_neighbors(astar_node.node);
      for( auto next_node : neighbors){
        if (parent.count(next_node) == 0 || dist[next_node] > astar_node.g + 1 ){
          open.push(AStarNode(next_node, astar_node.g + 1));
          parent[next_node] = astar_node.node;
        }
      }
    }
    Path p;
    if (parent.count(target) != 0 ){
      std::list<Node> path_list;
      Node current = target;
      path_list.push_front(current);
      while( current != source ){
        path_list.push_front(current);
        current = parent[current];
      }      
      for (auto node : path_list){
        p.PushBack(node);
      }
    }
    return p;
  }
  size_t getShortestPathDistance(const Node &source, const Node &target) override {
    // Use Dijkstra for nodes belonging to goal
    if (target_nodes_.find(target) != target_nodes_.end()){
      return dijkstra_.getShortestPathDistance(source, target);
    }
    // Use AStar otherwise
    if (dist_[source].count(target) > 0){
      return dist_[source][target];
    } else {
      size_t d = getShortestPath(source, target).size();
      dist_[source][target] = d;
      return d;
    }
  }
};

