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

void Execution::PushBack(std::shared_ptr<const Path> p) { exec_.push_back(p); }

Configuration Execution::get_configuration(uint64_t time) const {
  Configuration c;
  for (Agent a = 0; a < static_cast<Agent>(exec_.size()); a++) {
    c.PushBack(exec_[a]->GetAtTimeOrLast(time));
  }
  return c;
}

void Execution::set_path(Agent a, std::shared_ptr<const Path> p) {
  if (exec_.size() <= static_cast <size_t>(a)) exec_.resize(a + 1);
  exec_[a] = p;
}

const std::shared_ptr<const Path> Execution::get_path(Agent a) const { return exec_[a]; }

size_t Execution::size() const { return exec_.size(); }

std::ostream& operator<<(std::ostream& os, const Execution& e) {
  os << "{";
  for (auto it = e.exec_.cbegin(); it != e.exec_.cend(); ++it) {
    os << **it;
    if (std::distance(it, e.exec_.cend()) > 1) os << ", ";
  }
  os << "}";
  return os;
}
