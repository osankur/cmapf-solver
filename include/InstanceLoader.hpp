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

#include <sstream>
#include <vector>
#include <string>

#include <InstanceCreator.hpp>
#include <CMAPF.hpp>
#include <Options.hpp>
#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"

namespace instance {
template <class GraphMove, class GraphComm>
class InstanceLoader : public InstanceCreator<GraphMove, GraphComm> {
 public:
  InstanceLoader() {}
  virtual void Load() = 0;
};

class XMLInstanceLoader : public InstanceLoader<ExplicitGraph, ExplicitGraph> {
 private:
  const std::string file_path_;
  const std::string graph_folder_;
  const CollisionMode collision_mode_;

  struct XMLNode {
   private:
    const std::string id_;
    const float x_;
    const float y_;

   public:
    XMLNode(const std::string& id, float x, float y) : id_(id), x_(x), y_(y) {}
    const std::string& id() const { return id_; }
  };

  std::vector<XMLNode> nodes_;

 public:
  XMLInstanceLoader(const std::string& filepath, const std::string& location, const CollisionMode collision_mode);
  void Load() override;
};

class JSONInstanceLoader : public InstanceLoader<ExplicitGraph, ExplicitGraph> {
 private:
  const std::string file_path_;

 public:
  explicit JSONInstanceLoader(const std::string& filepath);
  void Load() override;
};

}  // namespace instance
