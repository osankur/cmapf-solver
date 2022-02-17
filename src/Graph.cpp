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

size_t ExplicitGraph::node_count() const { return adjacency_->size(); }

size_t ExplicitGraph::edge_count() const {
  size_t sum = 0;
  for (auto& edges : *adjacency_) sum += edges.size();
  return sum;
}

void ExplicitGraph::AddNode(Node n) {
  if (adjacency_->size() <= n) adjacency_->resize(n);
  adjacency_->insert(adjacency_->begin() + n, std::unordered_set<Node>());
}

void ExplicitGraph::AddEdge(Node source, Node target) { (*adjacency_)[source].insert(target); }

void ExplicitGraph::AddPosition(Node n, int x, int y) {
  if (positions_->size() <= n) positions_->resize(n);
  positions_->insert(positions_->begin() + n, std::make_pair(x, y));
}

std::pair<int,int> ExplicitGraph::get_position(Node node) const {
    return (*positions_)[node];
}

const std::unordered_set<Node>& ExplicitGraph::get_neighbors(Node n) const { return (*adjacency_)[n]; }

size_t ExplicitGraph::get_distance(Node a, Node b) const {
  return static_cast<size_t>(abs((*positions_)[a].first - (*positions_)[b].first) +
                             abs((*positions_)[a].second - (*positions_)[b].second));
}

bool ExplicitGraph::is_configuration_connected(const Configuration& config) const {
  int nb_agents = config.size();
    std::vector<bool> agent_treated =
        std::vector<bool>(nb_agents, false);
    std::stack<Agent> agent_stack;
    size_t agent_count = 0;
    std::vector<bool>::iterator it;

    agent_stack.push(0);
    while (!agent_stack.empty()) {
      Agent a = agent_stack.top();
      agent_stack.pop();

      if (!agent_treated[a]) {
        agent_treated[a] = true;
        agent_count++;

        Node aPos = config.at(a);

        for (Agent b = 0; b < static_cast<Agent>(nb_agents);
             b++) {
          if (a != b && !agent_treated[b]) {
            Node bPos = config.at(b);
            const auto& neighbors =
                get_neighbors(aPos);
            if (neighbors.find(bPos) != neighbors.end()) {
              agent_stack.push(b);
            }
          }
        }
      }
    }
    return agent_count == nb_agents;
  }
