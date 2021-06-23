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
#include <memory>
#include <set>
#include <map>
#include <stack>
#include <vector>
#include <utility>
#include <Instance.hpp>
#include <Execution.hpp>
#include <Configuration.hpp>

template <class GraphMove, class GraphComm>
bool IsConfigurationConnected(const Configuration& config, const Instance<GraphMove, GraphComm>& instance) {
  std::vector<bool> agent_treated = std::vector<bool>(instance.nb_agents(), false);
  std::stack<Agent> agent_stack;
  size_t agent_count = 0;
  std::vector<bool>::iterator it;

  while ((it = find(agent_treated.begin(), agent_treated.end(), false)) != agent_treated.end()) {
    agent_stack.push(distance(agent_treated.begin(), it));

    while (!agent_stack.empty()) {
      Agent a = agent_stack.top();
      agent_stack.pop();

      if (agent_treated[a]) continue;

      agent_treated[a] = true;
      agent_count++;

      Node aPos = config[a];

      for (Agent b = 0; b < static_cast<Agent>(instance.nb_agents()); ++b) {
        if (a != b && !agent_treated[b]) {
          Node bPos = config[b];
          const std::set<Node>& neighbors = instance.graph().communication().get_neighbors(aPos);

          if (neighbors.find(bPos) != neighbors.end()) {
            agent_stack.push(b);
          }
        }
      }
    }
  }
  return agent_count == instance.nb_agents();
}

template <class GraphMove, class GraphComm>
bool IsExecutionConnected(const Execution& exec, const Instance<GraphMove, GraphComm>& instance) {
  size_t max = 0;
  for (Agent agt = 0; agt < static_cast<Agent>(instance.nb_agents()); agt++)
    if (exec.get_path(agt)->size() > max) max = exec.get_path(agt)->size();

  for (size_t t = 0; t < max; t++)
    if (!IsConfigurationConnected(exec.get_configuration(t), instance)) return false;

  return true;
}
