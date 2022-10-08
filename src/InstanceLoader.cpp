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
#include <iostream>
#include <InstanceLoader.hpp>
#include <fstream>
#include <string>

namespace instance {


inline static bool file_exists (const std::string & name) {
    std::ifstream f(name.c_str());
    return f.good();
}

XMLInstanceLoader::XMLInstanceLoader(const std::string& filepath, const std::string& location, const CollisionMode collision_mode)
    : file_path_(filepath), graph_folder_(location), collision_mode_(collision_mode) {}

void XMLInstanceLoader::Load() {
  if (!file_exists(file_path_)) 
  {
    throw std::runtime_error("File " + std::string(file_path_) + " not found.");
  }
  std::ifstream infile(file_path_);

  std::string line;
  std::string type;
  std::string sMovFile;
  std::string sComFile;

  std::getline(infile, line);
  std::istringstream iss1(line);
  iss1 >> type >> sMovFile;
  sMovFile = graph_folder_ + sMovFile;
  std::getline(infile, line);
  std::istringstream iss2(line);
  iss2 >> type >> sComFile;
  sComFile = graph_folder_ + sComFile;

  char* movFile = sMovFile.data();
  char* comFile = sComFile.data();

  if (!file_exists(std::string(movFile))) 
  {
    throw std::runtime_error("File " + std::string(movFile) + " not found.");
  }
  if(!file_exists(std::string(comFile))){
    throw std::runtime_error("File " + std::string(comFile) + " not found.");
  }
  rapidxml::file<> xmlFile(movFile);
  rapidxml::xml_document<> movDoc;
  movDoc.parse<0>(xmlFile.data());

  const rapidxml::xml_node<>* movGraph = movDoc.first_node("graphml")->first_node("graph");

  // Creating the nodes
  for (const rapidxml::xml_node<>* node = movGraph->first_node("node"); node != NULL; node = node->next_sibling()) {
    rapidxml::xml_attribute<>* attr = node->first_attribute("id");
    if (attr == NULL) continue;
    std::string id(attr->value());
    id.erase(0, 1);
    const rapidxml::xml_node<>* data = node->first_node("data");
    float x = std::stof(data->value());
    data = data->next_sibling();
    float y = std::stof(data->value());
    const XMLNode n(id, x, y);

    nodes_.push_back(n);
    Node graphNode = (Node)std::stoi(id.c_str());
    instance_.graph().movement().AddNode(graphNode);
    instance_.graph().movement().AddEdge(graphNode, graphNode);
    instance_.graph().movement().AddPosition(graphNode, static_cast<int>(x), static_cast<int>(y));
    instance_.graph().communication().AddNode(graphNode);
    instance_.graph().communication().AddEdge(graphNode, graphNode);
  }

  // Creating the movement edges
  for (const rapidxml::xml_node<>* edge = movGraph->first_node("edge"); edge != NULL; edge = edge->next_sibling()) {
    rapidxml::xml_attribute<>* attr = edge->first_attribute("source");
    if (attr == NULL) continue;
    Node source = (Node)std::stoi(attr->value() + 1);
    attr = attr->next_attribute("target");
    Node target = (Node)std::stoi(attr->value() + 1);
    instance_.graph().movement().AddEdge(source, target);
    instance_.graph().movement().AddEdge(target, source);
  }

  xmlFile = rapidxml::file<>(comFile);
  rapidxml::xml_document<> comDoc;
  comDoc.parse<0>(xmlFile.data());

  rapidxml::xml_node<>* comGraph = comDoc.first_node("graphml")->first_node("graph");

  // Creating the communication edges
  for (rapidxml::xml_node<>* edge = comGraph->first_node("edge"); edge != NULL; edge = edge->next_sibling()) {
    rapidxml::xml_attribute<>* attr = edge->first_attribute("source");
    if (attr == NULL) continue;
    Node source = (Node)std::stoi(attr->value() + 1);
    attr = attr->next_attribute("target");
    Node target = (Node)std::stoi(attr->value() + 1);
    instance_.graph().communication().AddEdge(source, target);
    instance_.graph().communication().AddEdge(target, source);
  }

  std::string startConf;
  std::getline(infile, startConf);

  size_t startI = 0;
  auto endI = startConf.find(" ");

//  std::cerr << "Got whole: " << startConf << "\n";
  // Creating the start configuration
  while (endI != std::string::npos) {
    std::string s = startConf.substr(startI, endI - startI);
    // std::cerr << "Token: <" << s << ">\n";
    if (s != "start") instance_.start().push_back(std::stoi(s));
    startI = endI + 1;
    endI = startConf.find(" ", startI);
  }

  std::string goalConf;
  std::getline(infile, goalConf);

  startI = 0;
  endI = goalConf.find(" ");

  // Creating the goal configuration
  while (endI != std::string::npos) {
    std::string s = goalConf.substr(startI, endI - startI);
    if (s != "goal") instance_.goal().push_back(std::stoi(s));
    startI = endI + 1;
    endI = goalConf.find(" ", startI);
  }

  instance_.set_nb_agents(instance_.start().size());
  instance_.set_collision_mode(this->collision_mode_);
}

}  // namespace instance
