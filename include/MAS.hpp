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

#include <map>
#include <stack>
#include <limits>
#include <utility>
#include <Solver.hpp>
#include <LowLevel.hpp>

namespace decoupled {

template <class GraphMove, class GraphComm>
class MAS : public Solver<GraphMove, GraphComm> {
 private:
  std::map<std::pair<Node, size_t>, float> pheromone_map_;
  decoupled::low_level::NegativeAStar<GraphMove, GraphComm> astar;
  float evaporation_ = 0.8f;
  int max_iteration_ = 200;

 public:
  MAS(const Instance<GraphMove, GraphComm>& instance, const Objective& objective)
      : Solver<GraphMove, GraphComm>(instance, objective), astar(this->instance_) {
    Execution exec;
    for (Agent agt = 0; agt < static_cast<Agent>(this->instance_.nb_agents()); agt++) {
      Path p = astar.ComputeShortestPath(this->instance_.start()[agt], this->instance_.goal()[agt]);
      exec.PushBack(std::make_shared<const Path>(p));
    }
    this->execution_ = exec;
    UpdatePheromoneMap(exec);
  }

  void UpdatePheromoneMap(const Execution& exec) {
    std::map<std::pair<Node, size_t>, int> count_map;

    for (Agent i = 0; i < static_cast<Agent>(this->instance_.nb_agents()); i++) {
      auto path_i = exec.get_path(i);
      for (size_t time = 0; time < path_i->size(); time++) {
        Node pos_i = path_i->GetAtTimeOrLast(time);
        const std::set<Node>& neighbors = this->instance_.graph().communication().get_neighbors(pos_i);
        for (Node n : neighbors) {
          auto found = count_map.find(std::make_pair(n, time));
          if (found != count_map.end())
            found->second++;
          else
            count_map.insert(std::make_pair(std::make_pair(n, time), 1));
        }
      }
    }
    // Evaporate
    for (auto& pair : pheromone_map_) {
      pair.second = (1 - evaporation_) * pair.second;
    }
    // Lay now pheromones
    for (auto& pair : count_map) {
      auto found = pheromone_map_.find(pair.first);
      if (found != pheromone_map_.end())
        found->second += pair.second;
      else
        pheromone_map_.insert(std::make_pair(pair.first, static_cast<float>(pair.second)));
    }
  }

  Node SelectStep(Agent agt, Node current, size_t time) {
    size_t good_strategy = (size_t)rand() % 2;
    const std::set<Node>& neighbors = this->instance_.graph().movement().get_neighbors(current);
    if (good_strategy == 0) {
      double choice = (double)rand() / std::numeric_limits<int>::max();
      float sum_pheromone = 0.0f;
      for (auto n : neighbors) {
        auto found = pheromone_map_.find(std::make_pair(n, time));
        if (found != pheromone_map_.end())
          sum_pheromone += found->second *
                           (1 / (this->instance_.graph().movement().get_distance(n, this->instance_.goal()[agt]) + 1));
      }
      float current_pheromone = 0.0f;
      for (auto n : neighbors) {
        auto found = pheromone_map_.find(std::make_pair(n, time));
        float pheromone_value = current_pheromone;
        if (found != pheromone_map_.end())
          pheromone_value +=
              found->second *
              (1 / (this->instance_.graph().movement().get_distance(n, this->instance_.goal()[agt]) + 1)) /
              sum_pheromone;
        if (choice <= pheromone_value) return n;
        current_pheromone = pheromone_value;
      }
    } else {
      Path p = astar.ComputeShortestPath(current, this->instance_.goal()[agt]);
      return p[1];
    }

    // No pheromone pick random
    size_t random_strategy = (size_t)rand() % 2;
    if (random_strategy == 0) {
      Node min_node = std::numeric_limits<Node>::max();
      size_t min_dist = std::numeric_limits<size_t>::max();
      for (auto n : neighbors) {
        size_t current_dist = this->instance_.graph().movement().get_distance(n, this->instance_.goal()[agt]);
        if (current_dist < min_dist) {
          min_dist = current_dist;
          min_node = n;
        }
      }
      if (min_node == std::numeric_limits<Node>::max()) throw "Potential error of node selection";
      return min_node;
    } else {
      size_t default_choice = (size_t)rand() % neighbors.size();
      auto it = neighbors.begin();
      std::advance(it, default_choice);
      return *it;
    }
  }

  size_t CountDisconnection(const Execution& exec) {
    size_t count = 0;
    size_t exec_max_size = 0;

    for (Agent agt = 0; agt < static_cast<Agent>(this->instance_.nb_agents()); agt++)
      if (exec_max_size < exec.get_path(agt)->size()) exec_max_size = exec.get_path(agt)->size();

    for (size_t time = 0; time < exec_max_size; time++) {
      siez_t agent_count = 0;
      std::stack<Agent> agent_stack;
      std::vector<bool> agent_treated = std::vector<bool>(this->instance_.nb_agents(), false);
      agent_stack.push(0);

      while (!agent_stack.empty()) {
        Agent a = agent_stack.top();
        agent_stack.pop();
        if (agent_treated[a]) continue;
        agent_treated[a] = true;
        agent_count++;

        Node aPos = exec.get_path(a)->GetAtTimeOrLast(time);

        for (Agent b = 0; b < static_cast<Agent>(this->instance_.nb_agents()); ++b) {
          if (a != b && !agent_treated[b]) {
            Node bPos = exec.get_path(b)->GetAtTimeOrLast(b);
            const std::set<Node>& neighbors = this->instance_.graph().communication().get_neighbors(aPos);

            if (neighbors.find(bPos) != neighbors.end()) {
              agent_stack.push(b);
            }
          }
        }
      }
      if (agent_count != this->instance_.nb_agents()) count++;
    }
    return count;
  }

  bool StepCompute() override {
    Execution exec;
    size_t previous_disc = CountDisconnection(this->execution_);
    if (previous_disc == 0) return true;

    for (Agent agt = 0; agt < static_cast<Agent>(this->instance_.nb_agents()); agt++) {
      Path path_agt;
      path_agt.PushBack(this->instance_.start()[agt]);

      while (path_agt[path_agt.size() - 1] != this->instance_.goal()[agt]) {
        Node next_node = SelectStep(agt, path_agt[path_agt.size() - 1], path_agt.size());
        path_agt.PushBack(next_node);
      }
      exec.PushBack(std::make_shared<const Path>(path_agt));
    }

    if (this->objective_.cost(exec) < this->objective_.cost(this->execution_) ||
        CountDisconnection(exec) < previous_disc) {
      std::cout << "Iteration: " << max_iteration_ << std::endl;
      std::cout << "Execution Size: " << this->objective_.cost(exec) << std::endl;
      std::cout << "Disconnection Count: " << CountDisconnection(exec) << std::endl;
      this->execution_ = exec;
      
    }
    UpdatePheromoneMap(exec);
    max_iteration_--;
    if (max_iteration_ == 0) return true;

    return false;
  }
};

}  // namespace decoupled