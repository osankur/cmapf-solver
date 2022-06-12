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
#include <Execution.hpp>

Configuration Execution::getConfiguration(uint64_t time) const {
  Configuration c;
  for (Agent a = 0; a < static_cast<Agent>(this->size()); a++) {
    c.push_back(this->at(a)->getAtTimeOrLast(time));
  }
  return c;
}

void Execution::setPath(Agent a, std::shared_ptr<const Path> p) {
  if (this->size() <= static_cast<size_t>(a)) this->resize(a + 1);
  (*this)[a] = p;
}


void Execution::setPaths(const std::vector<std::shared_ptr<Configuration>> & confseq){
  this->clear();
  if (confseq.size() == 0) return;
  size_t nb_agents = confseq[0]->size();
  for (size_t agt = 0; agt < nb_agents; agt++)
  {
    std::shared_ptr<Path> p_agt = std::make_shared<Path>();
    for (auto conf : confseq){
      p_agt->push_back(conf->at(agt));
    }
    this->setPath(agt, p_agt);
  }
}

const std::shared_ptr<const Path> Execution::getPath(Agent a) const { return this->at(a); }



size_t Execution::maxPathLength() const {
  size_t max = 0;
  for (auto& path : *this)
    if (max < path->size()) max = path->size();
  return max;
}

std::ostream& operator<<(std::ostream& os, const Execution& e) {
  os << "{";
  for (auto it = e.cbegin(); it != e.cend(); ++it) {
    os << **it;
    if (std::distance(it, e.cend()) > 1) os << ", ";
  }
  os << "}";
  return os;
}
