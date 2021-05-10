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
#include <string>
#include <Configuration.hpp>

void Configuration::PushBack(Node node) { config_.push_back(node); }

Node& Configuration::operator[](Agent a) { return config_[a]; }

const Node& Configuration::operator[](Agent a) const { return config_[a]; }

size_t Configuration::size() const { return config_.size(); }

// Friends

std::ostream& operator<<(std::ostream& os, const Configuration& config) {
  os << "<";
  for (auto it = config.config_.cbegin(); it != config.config_.cend(); ++it) {
    os << std::to_string(*it);
    if (std::distance(it, config.config_.cend()) > 1) os << ", ";
  }
  os << ">";
  return os;
}
