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

class Execution {
 private:
  std::vector<std::shared_ptr<const Path>> exec_;

 public:
  void PushBack(std::shared_ptr<const Path>);

  Configuration get_configuration(uint64_t) const;

  void set_path(Agent, std::shared_ptr<const Path>);
  const std::shared_ptr<const Path> get_path(Agent) const;

  // Properties
  size_t size() const;
  size_t max_path() const;

  // Friends
  friend std::ostream& operator<<(std::ostream&, const Execution&);
};

