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
#include "AppConfig.h"

using namespace boost::program_options;

constexpr char DEFAULT_ALG[] = "CA";
constexpr char DEFAULT_OBJ[] = "SUM";

enum class Algorithm : int { CBS = 0, CCBS, CA, MAS, DFS };
enum class ObjectiveEnum : int { SUM = 0, MAX };

int main(int argc, const char* argv[]) {
  try {
    options_description desc{"Options"};
    desc.add_options()("help,h", "Help screen")("experience,e", value<std::string>(), "The experience file to run.")(
        "graph-folder,G", value<std::string>(), "The graph folder.")(
        "algo,a", value<std::string>()->default_value(DEFAULT_ALG), "The algorthm to run.")(
        "objective,O", value<std::string>()->default_value(DEFAULT_OBJ), "The objective to minimize");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help")) {
      std::cout << desc << '\n';
      return 0;
    }
#ifdef DEBUG
    instance::XMLInstanceLoader il(
        std::string(std::string(PROJECT_SOURCE_DIR) + "\\data\\b-w-open_uniform_grid_13_range_150_10_0.exp"),
        std::string(std::string(PROJECT_SOURCE_DIR) + "\\data\\"));
#else

    if (!vm.count("experience")) {
      LOG_FATAL("The experience is missing !");
      return 1;
    } else {
      LOG_INFO("Experience: " << vm["experience"].as<std::string>());
    }

    if (!vm.count("graph-folder")) {
      LOG_FATAL("The graph-folder is missing !");
      return 1;
    } else {
      LOG_INFO("Graph Folder: " << vm["graph-folder"].as<std::string>());
    }

    instance::XMLInstanceLoader il(vm["experience"].as<std::string>(), vm["graph-folder"].as<std::string>());
#endif
    LOG_TRACE("Instance created!");

    il.Load();

    LOG_TRACE("Instance loaded!");

    auto obj = magic_enum::enum_cast<ObjectiveEnum>(std::string(DEFAULT_OBJ));
    if (vm.count("objective")) {
      obj = magic_enum::enum_cast<ObjectiveEnum>(vm["objective"].as<std::string>());
      if (!obj.has_value()) {
        LOG_FATAL("The objective " << vm["objective"].as<std::string>() << " is unkown !");
        return 1;
      }
    }

    LOG_INFO("Objective :" << vm["objective"].as<std::string>());

    std::unique_ptr<Objective> objective = nullptr;

    switch (obj.value()) {
      case ObjectiveEnum::SUM:
        objective = std::make_unique<SumObjective>();
        break;
      case ObjectiveEnum::MAX:
        objective = std::make_unique<MaxObjective>();
        break;
    }

    LOG_TRACE("Objective created!");

    auto algo = magic_enum::enum_cast<Algorithm>(std::string(DEFAULT_ALG));
    if (vm.count("algo")) {
      algo = magic_enum::enum_cast<Algorithm>(vm["algo"].as<std::string>());
      if (!algo.has_value()) {
        LOG_FATAL("The algorithm " << vm["algo"].as<std::string>() << " is unkown !");
        return 1;
      }
    }

    LOG_INFO("Algorithm :" << vm["algo"].as<std::string>());

    std::unique_ptr<Solver<ExplicitGraph, ExplicitGraph>> solver = nullptr;

    switch (algo.value()) {
      case Algorithm::CBS: {
        decoupled::ctn_ordering::LeastConflictStrategy ord;
        decoupled::conflict_selection::FirstConflictStrategy con;
        solver = std::make_unique<decoupled::high_level::CBS<ExplicitGraph, ExplicitGraph>>(il.instance(),
                                                                                            *objective.get(), ord, con);
        break;
      }
      case Algorithm::CCBS: {
        decoupled::ctn_ordering::LeastConflictStrategy ord;
        decoupled::conflict_selection::FirstConflictStrategy con;
        solver =
            std::make_unique<decoupled::high_level::CCBS<ExplicitGraph>>(il.instance(), *objective.get(), ord, con);
        break;
      }
      case Algorithm::CA:
        solver = std::make_unique<decoupled::CAStar<ExplicitGraph, ExplicitGraph>>(il.instance(), *objective.get(), 3);
        break;
      case Algorithm::MAS:
        solver = std::make_unique<decoupled::MAS<ExplicitGraph, ExplicitGraph>>(il.instance(), *objective.get());
        break;
      case Algorithm::DFS:
        solver = std::make_unique<coupled::DFS<ExplicitGraph, ExplicitGraph>>(il.instance(), *objective.get());
        break;
    }

    LOG_TRACE("Solver created!");

    const auto execution = solver->Compute();

    LOG_TRACE("Solver terminated!");

    LOG_INFO("Execution cost:" << objective->cost(execution));
    LOG_INFO("Execution:" << execution);

  } catch (const error& ex) {
    std::cerr << ex.what() << '\n';
  }
}