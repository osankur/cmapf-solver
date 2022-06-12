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
#include <Path.hpp>
#include <memory>
#include <unordered_set>
#include <Logger.hpp>
#include <iostream>
#include <Graph.hpp>

Path::Path() {}

Path::Path(const Path& rhs) : std::vector<Node>(*this) {}

//void Path::Resize(size_t time) { path_.resize(time); }

// void Path::PushBack(Node node) { path_.push_back(node); }

const Node& Path::getAtTimeOrLast(size_t time) const { return time < this->size() ? (*this)[time] : this->back(); }

// Node& Path::operator[](size_t time) { return path_[time]; }

// const Node& Path::operator[](size_t time) const { return path_[time]; }

// size_t Path::size() const { return path_.size(); }

// const Node& Path::at(size_t time) const { return path_[time]; }

bool Path::isValid(const ExplicitGraph & graph) const {
  if (size() == 0) return true;
  Node n = this->at(0);
  for (size_t i = 1; i < size(); i++){
    Node next_n = this->at(i);
    const std::unordered_set<Node>& neighbors = graph.get_neighbors(n);
    if (neighbors.find(next_n) == neighbors.end()){
      std::cerr << "The following path is not valid\n" << *this << "\n";
      return false;
    }
    n = next_n;
  }
  return true;
}


// Friends

std::ostream& operator<<(std::ostream& os, const Path& path) {
  os << "[";
  for (auto it = path.cbegin(); it != path.cend(); ++it) {
    os << std::to_string(*it);
    if (std::distance(it, path.cend()) > 1) os << ", ";
  }
  os << "]";
  return os;
}

