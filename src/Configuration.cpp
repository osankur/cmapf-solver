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
#include <set>
#include <Configuration.hpp>

bool Configuration::operator==(const Configuration &c) const
{
  int n = this->size();
  if (n != c.size())
    return false;
  for (int i = 0; i < n; i++)
  {
    if (c[i] != this->at(i))
      return false;
  }
  return true;
}
bool Configuration::hasCollisions() const {
  std::set<Node> support(this->begin(), this->end());
  return support.size() < this->size();
}



// Friends

std::ostream &operator<<(std::ostream &os, const Configuration &config)
{
  os << "<";
  for (auto it = config.cbegin(); it != config.cend(); ++it)
  {
    os << std::to_string(*it);
    if (std::distance(it, config.cend()) > 1)
      os << ", ";
  }
  os << ">";
  return os;
}

bool operator<(const Configuration &c1, const Configuration &c2)
{
  return static_cast<std::vector<Node>>(c1)< static_cast<std::vector<Node>>(c2);
}
