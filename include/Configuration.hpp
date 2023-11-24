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

#include <ostream>
#include <vector>
#include <memory>
#include <CMAPF.hpp>

class Configuration : public std::vector<Node>
{
public:
  Configuration() : std::vector<Node>(){}
  Configuration(size_t size) : std::vector<Node>(){
    this->resize(size);
  }

  bool operator==(const Configuration &) const;
  bool hasCollisions() const;
  // Friends
  friend std::ostream &operator<<(std::ostream &, const Configuration &);
};

bool operator<(const Configuration &c1, const Configuration &c2);

struct ConfigurationPtrEqual
{
  bool operator()(const std::shared_ptr<Configuration> &a,
                  const std::shared_ptr<Configuration> &b) const
  {
    if (a->size() != b->size())
      return false;

    for (size_t index = 0; index < a->size(); index++)
      if (a->at(index) != b->at(index))
        return false;
    return true;
  }
};

struct ConfigurationPtrHash
{
  size_t operator()(const std::shared_ptr<Configuration> &that) const
  {
    size_t h = 0;
    for (size_t index = 0; index < that->size(); index++)
      h += that->at(index) ^ index;

    return h;
  }
};
