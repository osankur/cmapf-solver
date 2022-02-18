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
#ifdef _MSC_VER
#pragma warning(disable : 4459)
#endif

#include <Logger.hpp>
#include <Objective.hpp>
#include <InstanceLoader.hpp>
#include <magic_enum.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <CBS.hpp>
#include <CCBS.hpp>
#include <CAStar.hpp>
#include <MAS.hpp>
#include <DFS.hpp>
#include <CoordSolver.hpp>
#include <CMARRT.hpp>
#include "AppConfig.h"

using namespace boost::program_options;

constexpr char DEFAULT_ALG[] = "CCBS";
constexpr char DEFAULT_OBJ[] = "SUM";
constexpr char DEFAULT_HEURISTICS[] = "BIRDEYE";
constexpr char DEFAULT_COLLISIONS[] = "IGNORE_COLLISIONS";
constexpr char DEFAULT_SUBSOLVER[] = "DFS_SOLVER";

enum class Algorithm : int
{
  CBS = 0,
  CCBS,
  CA,
  MAS,
  DFS,
  COORD,
  CMARRT
};
enum class ObjectiveEnum : int
{
  SUM = 0,
  MAX
};

enum class HeuristicsEnum : int
{
  SHORTEST_PATH,
  BIRDEYE
};

int main(int argc, const char *argv[])
{
  try
  {
    options_description desc{"Options"};
    desc.add_options()("help,h", "Help screen")("experience,e", value<std::string>(), "The experience file to run.")(
        "graph-folder,G", value<std::string>(), "The graph folder.")(
        "algo,a", value<std::string>()->default_value(DEFAULT_ALG), "The algorthm to run.")(
        "window,w", value<int>()->default_value(2), "Window size.")(
        "objective,O", value<std::string>()->default_value(DEFAULT_OBJ), "The objective to minimize")(
        "collisions,c", value<std::string>()->default_value(DEFAULT_COLLISIONS), "Whether to check for collisions: CHECK_COLLISIONS | IGNORE_COLLISIONS")(
        "heuristics,h", value<std::string>()->default_value(DEFAULT_HEURISTICS), "The heuristics to be used in DFS and CMARRT: BIRDEYE or SHORTEST_PATH")(
        "subsolver,ss", value<std::string>()->default_value(DEFAULT_SUBSOLVER), "Which solver to use to generate segment paths in CMARRT: DECOUPLED_SOLVER | DFS_SOLVER | COORD_SOLVER.")(
        "random_seed,rs", value<int>()->default_value(-1), "Seed for the random generator.")(
        "prob2goal,p", value<int>()->default_value(50), "In CMARRT, the probability expressed as a percentage in [0,100] of picking goal as a target. Default is 50%.")(
        "step_size,s", value<int>()->default_value(10), "In CMARRT, the number of steps of the expanding path to create the new node expanding the tree.");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
    {
      std::cout << desc << '\n';
      return 0;
    }

    if (!vm.count("experience"))
    {
      LOG_FATAL("The experience is missing !");
      return 1;
    }
    else
    {
      LOG_INFO("Experience: " << vm["experience"].as<std::string>());
    }

    if (!vm.count("graph-folder"))
    {
      LOG_FATAL("The graph-folder is missing !");
      return 1;
    }
    else
    {
      LOG_INFO("Graph Folder: " << vm["graph-folder"].as<std::string>());
    }

    instance::XMLInstanceLoader il(vm["experience"].as<std::string>(), vm["graph-folder"].as<std::string>());

    LOG_TRACE("Instance created!");

    il.Load();

    LOG_TRACE("Instance loaded!");
    LOG_TRACE("Number of nodes: " + std::to_string(il.instance().graph().movement().node_count()));
    LOG_TRACE("Number of edges: " + std::to_string(il.instance().graph().movement().edge_count()));
    LOG_TRACE("Number of agents: " + std::to_string(il.instance().start().size()));

    auto cgoal = il.instance().goal();
    auto cstart = il.instance().start();
    if (!il.instance().graph().communication().is_configuration_connected(cstart))
    {
      LOG_FATAL("Start configuration is not connected.");
      throw "Error";
    }
    if (!il.instance().graph().communication().is_configuration_connected(cgoal))
    {
      LOG_FATAL("Goal configuration is not connected.");
      throw "Error";
    }

    auto obj = magic_enum::enum_cast<ObjectiveEnum>(std::string(DEFAULT_OBJ));
    if (vm.count("objective"))
    {
      obj = magic_enum::enum_cast<ObjectiveEnum>(vm["objective"].as<std::string>());
      if (!obj.has_value())
      {
        LOG_FATAL("The objective " << vm["objective"].as<std::string>() << " is unkown !");
        return 1;
      }
    }

    LOG_INFO("Objective :" << vm["objective"].as<std::string>());

    std::unique_ptr<Objective> objective = nullptr;

    switch (obj.value())
    {
    case ObjectiveEnum::SUM:
      objective = std::make_unique<SumObjective>();
      break;
    case ObjectiveEnum::MAX:
      objective = std::make_unique<MaxObjective>();
      break;
    }

    LOG_TRACE("Objective created!");

    auto algo = magic_enum::enum_cast<Algorithm>(std::string(DEFAULT_ALG));
    if (vm.count("algo"))
    {
      algo = magic_enum::enum_cast<Algorithm>(vm["algo"].as<std::string>());
      if (!algo.has_value())
      {
        LOG_FATAL("The algorithm " << vm["algo"].as<std::string>() << " is unkown !");
        return 1;
      }
    }

      LOG_INFO("Heuristics:" << vm["heuristics"].as<std::string>());
      auto floydwarshall = std::make_shared<FloydWarshall<ExplicitGraph, ExplicitGraph>>(il.instance());
      std::unique_ptr<Heuristics<ExplicitGraph, ExplicitGraph>> heuristics = nullptr;
      auto heuristics_mode = magic_enum::enum_cast<HeuristicsEnum>(vm["heuristics"].as<std::string>());
      switch (heuristics_mode.value())
      {
      case HeuristicsEnum::BIRDEYE:
        heuristics = std::make_unique<BirdEyeHeuristics<ExplicitGraph, ExplicitGraph>>(il.instance(), floydwarshall);
        break;
      case HeuristicsEnum::SHORTEST_PATH:
        heuristics = std::make_unique<ShortestPathHeuristics<ExplicitGraph, ExplicitGraph>>(il.instance(), floydwarshall);
        break;
      }

    LOG_INFO("Collisions:" << vm["collisions"].as<std::string>());
    auto collision_mode = magic_enum::enum_cast<CollisionsEnum>(vm["collisions"].as<std::string>()).value();

    LOG_INFO("Subsolver:" << vm["subsolver"].as<std::string>());
    SubsolverEnum subsolver = magic_enum::enum_cast<SubsolverEnum>(vm["subsolver"].as<std::string>()).value();

    LOG_INFO("Algorithm:" << vm["algo"].as<std::string>());

    std::unique_ptr<Solver<ExplicitGraph, ExplicitGraph>> solver = nullptr;

    switch (algo.value())
    {
    case Algorithm::CBS:
    {
      decoupled::conflict_selection::FirstConflictStrategy con;
      solver = std::make_unique<
          decoupled::high_level::CBS<ExplicitGraph, ExplicitGraph, decoupled::ctn_ordering::LeastConflictStrategy>>(
          il.instance(), *objective.get(), con);
      break;
    }
    case Algorithm::CCBS:
    {
      decoupled::conflict_selection::FirstConflictStrategy con;
      solver = std::make_unique<
          decoupled::high_level::CCBS<ExplicitGraph, decoupled::ctn_ordering::LeastConflictStrategy>>(
          il.instance(), *objective.get(), con);
      break;
    }
    case Algorithm::CA:
      solver = std::make_unique<decoupled::CAStar<ExplicitGraph, ExplicitGraph>>(il.instance(), *objective.get(), 3);
      break;
    case Algorithm::MAS:
      solver = std::make_unique<decoupled::MAS<ExplicitGraph, ExplicitGraph>>(il.instance(), *objective.get());
      break;
    case Algorithm::DFS:
      solver = std::make_unique<coupled::DFS<ExplicitGraph, ExplicitGraph>>(il.instance(), *objective.get(), *heuristics.get());
      break;
    case Algorithm::COORD:
      solver = std::make_unique<coordinated::CoordSolver<ExplicitGraph, ExplicitGraph>>(il.instance(),
                                                                                        *objective.get(),
                                                                                        vm["window"].as<int>(),
                                                                                        collision_mode);
      break;
    case Algorithm::CMARRT:
      int random_seed = vm["random_seed"].as<int>();
      if (random_seed < 0)
      {
        srand(time(NULL));
      }
      else
      {
        srand(random_seed);
      }
      LOG_INFO("Prob2goal:" << vm["prob2goal"].as<int>());
      LOG_INFO("Step size:" << vm["step_size"].as<int>());

      solver = std::make_unique<cmarrt::CMARRT<ExplicitGraph, ExplicitGraph>>(
          il.instance(),
          *objective.get(),
          *heuristics.get(),
          floydwarshall,
          subsolver,
          collision_mode,
          vm["prob2goal"].as<int>(),
          vm["step_size"].as<int>());
      break;
    }

    LOG_TRACE("Solver created!");

    const auto execution = solver->compute();

    LOG_TRACE("Solver terminated!");
    LOG_INFO("Execution cost:" << objective->cost(execution));
    LOG_INFO("Execution:" << execution);
  }
  catch (const error &ex)
  {
    std::cerr << ex.what() << '\n';
  }
}