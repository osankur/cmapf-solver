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

#include <Instance.hpp>

namespace instance {

template <class GraphMove, class GraphComm>
class InstanceCreator {
 protected:
  Instance<GraphMove, GraphComm> instance_;

 public:
  const Instance<GraphMove, GraphComm>& instance() const { return instance_; }
};

template <class GraphMove, class GraphComm>
class InteractiveInstanceCreator : public InstanceCreator<GraphMove, GraphComm> {};

}  // namespace instance
