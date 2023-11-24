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
#include <Objective.hpp>
#include <Solver.hpp>
#include <PriorityQueue.hpp>
#include <Heuristics.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <BoundedSolver.hpp>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <stack>
#include <unordered_set>
#include <utility>
#include <vector>
#include <ctime>

namespace coupled
{

  template <class GraphMove, class GraphComm>
  class DFS : public Solver<GraphMove, GraphComm>, public BoundedSolver<GraphMove,GraphComm>
  {
  private:
    std::vector<std::shared_ptr<Configuration>> exec_;
    std::unordered_set<std::shared_ptr<Configuration>,
                       ConfigurationPtrHash,
                       ConfigurationPtrEqual>
        closed_;
    Heuristics<GraphMove, GraphComm> & heuristics_;
    bool verbose_;
    int max_iterations_;
    int iteration_count_;

    typedef std::map<std::shared_ptr<Configuration>, double> PartialCostMap;
    typedef PriorityQueue<std::shared_ptr<Configuration>> PartialConfQueue;
      static bool isConfigurationWindowConnected(const Configuration &c, const Instance<GraphMove, GraphComm> &instance, int window_size)
      {
          Configuration localView;
          int local_index = 0;
          for (size_t i = 0; i < instance.nb_agents(); i++)
          {
              localView.push_back(c.at(i));
              if ( (i % (window_size-1)) == 0)
              {
                  if (!instance.graph().communication().isConfigurationConnected(localView))
                  {
                      return false;
                  }
                  localView.clear();
                  localView.push_back(c.at(i));
                  local_index = 0;
              }
              local_index++;
          }
          return true;
      }
    /**
     * @brief This is an alternative implementation of FindBestChild using Tateo's priority queue.
     *
     * @param config
     * @return std::shared_ptr<Configuration>
     */
    std::shared_ptr<Configuration> FindBestConfiguration(
        const std::shared_ptr<Configuration> pi,
        const Configuration & goal)
    {
      assert(this->instance_.nb_agents() == pi->size());
      auto cstart = clock();

      PartialCostMap g_local;
      PartialConfQueue open_local;

      auto pi0 = std::make_shared<Configuration>();
      g_local[pi0] = 0;
      open_local.insert(pi0, 0, heuristics_.getHeuristic(*pi, goal));
      while (!open_local.empty())
      {
        this->iteration_count_++;
        size_t a_cost;
        size_t a_g;
        auto a = open_local.pop(a_g, a_cost);
        if (this->max_iterations_ > 0 && this->iteration_count_ > this->max_iterations_){
          break;
        }
        
        if (a->size() == this->instance_.nb_agents() && 
            this->instance_.graph().communication().isConfigurationConnected(*a) && 
            (closed_.find(a) == closed_.end()) && 
            !((*a) == (*pi)) &&
            (
                this->instance().getCollisionMode() == CollisionMode::IGNORE_COLLISIONS
                || std::set<Node>(a->begin(), a->end()).size() == this->instance().nb_agents()
            )
          )
        {
          if (verbose_ && this->iteration_count_ > 1000){
            std::cout << ANSI_RED << "\tFindBestConfiguration ended after " << this->iteration_count_ << " iterations\n" << ANSI_RESET;
          }
          if (verbose_){
            std::cout << *a;
            std::cout << "\n";
            std::cout << "\ncost: " << a_cost << ", g=" << a_g << "\n";
            std::cout.flush();
          }
          return a;
        }
        else if (a->size() < pi->size())
        {
          Agent next_agt = (Agent)a->size();

          for (Node neighbor : this->instance_.graph().movement().get_neighbors(
                   pi->at(next_agt)))
          {
            // std::cout << "Setting agent " << next_agt << " to node " << neighbor << "\n";
            std::shared_ptr<Configuration> next = std::make_shared<Configuration>();
            for (Agent agt = 0; agt < next_agt; agt++){
              next->push_back(a->at(agt));
            }
            next->push_back(neighbor);
            double g = g_local[a];
            if (neighbor != pi->at(next_agt))
            {
              g += 1;
            }
            g_local[next] = g;

            Configuration completed_next(*next);
            for(Agent agt = next->size(); agt < pi->size(); agt++){
              completed_next.push_back(pi->at(agt));
            }
            open_local.insert(next, g, this->heuristics_.getHeuristic(completed_next, goal));
          }
        }
      }
      return nullptr;
    }

    /**
     * @brief execute one step of computation of DFS from [Tateo et al.] towards given custom goal
     *
     * @return true if the work is finished (we then found a solution)
     * @return false if the work is not finished
     */
    bool StepCompute(const Configuration & goal)
    {
      if (exec_.empty())
        return true;
      if ((*exec_.back()) == goal)
      {
        this->execution_.setPaths((exec_));
        return true;
      }
      this->iteration_count_ = 0;
      std::shared_ptr<Configuration> config = FindBestConfiguration(exec_.back(), goal);

      if (config == nullptr)
      {
        exec_.pop_back();
      }
      else
      {
        closed_.insert(config);
        exec_.push_back(config);
      }
      return false;
    }


  public:
    DFS(const Instance<GraphMove, GraphComm> &instance,
        const Objective &objective,
        Heuristics<GraphMove, GraphComm>& heuristics,
        bool verbose = true)
        : Solver<GraphMove, GraphComm>(instance, objective), exec_(), heuristics_(heuristics), verbose_(verbose)
    {
      std::shared_ptr<Configuration> start = std::make_shared<Configuration>();
      for (size_t agt = 0; agt < instance.start().size(); agt++)
        start->push_back(instance.start()[agt]);
      exec_.push_back(start);
      closed_.insert(start);
    }

    /**
     * @brief execute one step of computation of DFS from [Tateo et al.]
     *
     * @return true if the work is finished (we then found a solution)
     * @return false if the work is not finished
     */
    bool StepCompute()
    {
      StepCompute(this->instance_.goal());
    }

    const Execution compute() override {
      this->iteration_count_ = 0;
      int count = 0;
      // while (!StepCompute() && count++ < 10);
      while (!StepCompute());
      return this->execution_;
    }


    /**
     * @brief compute an execution that goes
     * `steps` steps from source towards goal (but not necessary reaching the goal).
     *
     * @return computed partial execution towards goal
     */
    std::vector<std::shared_ptr<Configuration> > computeBoundedPathTowards(const Configuration & source, const Configuration & goal, int steps = -1, int max_iterations=-1) override
    {
      exec_.clear();
      exec_.push_back(std::make_shared<Configuration>(source));
      closed_.clear();
      this->max_iterations_ = max_iterations;
      this->iteration_count_ = 0;
      // closed_.insert(std::make_shared<Configuration>(source));

      for (int i = 0; steps == - 1 || i < steps; i++)
      {
        if (StepCompute(goal)) break;
        if (this->max_iterations_ > 0 && this->iteration_count_ > this->max_iterations_)
          break;

      }
      return exec_;
    }
  };

} // namespace coupled
