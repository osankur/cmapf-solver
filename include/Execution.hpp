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

#include <memory>
#include <vector>
#include <Configuration.hpp>
#include <Path.hpp>

class Execution : public std::vector<std::shared_ptr<const Path>> {
 public:
  Configuration getConfiguration(uint64_t) const;

  void setPath(Agent, std::shared_ptr<const Path>);
  void setPaths(const std::vector<std::shared_ptr<Configuration>> & );
  const std::shared_ptr<const Path> getPath(Agent) const;

  // Properties
  size_t maxPathLength() const;

  // Friends
  friend std::ostream& operator<<(std::ostream&, const Execution&);
};

