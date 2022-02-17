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

#include <CTNOrderingStrategy.hpp>
#include <ConflictSelectionStrategy.hpp>
#include <ConstraintTreeNode.hpp>
#include <FloydWarshall.hpp>
#include <Instance.hpp>
#include <LowLevel.hpp>
#include <Objective.hpp>
#include <Solver.hpp>
#include <PriorityQueue.hpp>
#include <ShortestPathHeuristics.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <stack>
#include <unordered_set>
#include <utility>
#include <vector>

namespace coupled
{

  template <class GraphMove, class GraphComm>
  class DFS : public Solver<GraphMove, GraphComm>
  {
  private:
    std::vector<std::shared_ptr<Configuration>> exec_;
    std::unordered_set<std::shared_ptr<Configuration>,
                       ConfigurationPtrHash,
                       ConfigurationPtrEqual>
        closed_;
    ShortestPathHeuristics<GraphMove, GraphComm> shortestPathHeuristics_;

    typedef std::map<std::shared_ptr<Configuration>, double> PartialCostMap;
    typedef PriorityQueue<std::shared_ptr<Configuration>> PartialConfQueue;

    /**
     * @brief This is an alternative implementation of FindBestChild using Tateo's priority queue.
     *
     * @param config
     * @return std::shared_ptr<Configuration>
     */
    std::shared_ptr<Configuration> FindBestConfiguration(
        const std::shared_ptr<Configuration> pi)
    {
      assert(this->instance_.nb_agents() == pi->size());
      PartialCostMap g_local;
      PartialConfQueue open_local;

      auto pi0 = std::make_shared<Configuration>();
      g_local[pi0] = 0;
      open_local.insert(pi0, 0, shortestPathHeuristics_.getHeuristic(*pi));
      while (!open_local.empty())
      {
        auto a = open_local.pop();
        // std::cout << "\n* FindBestConfiguration Iteration. Open.size(): " << open_local.size() << ". Popped: ";
        // std::cout << *a;
        // std::cout << "\n";
        if (a->size() == this->instance_.nb_agents() && this->instance_.graph().communication().is_configuration_connected(*a) && (closed_.find(a) == closed_.end()) && !((*a) == (*pi)))
        {
          // std::cout << "open.size() == " << open_local.size() << " closed.size() == " << closed_.size() << "\n";
          // std::cout.flush();
          return a;
        }
        else if (a->size() < pi->size())
        {
          Agent next_agt = (Agent)a->size();
          // std::cout << "Agent " << next_agt << " (out of " << pi->size() << ") is at node " << pi->at(next_agt) << " and has "
          //           << this->instance_.graph().movement().get_neighbors(pi->at(next_agt)).size()
          //           << " neighbors\n";

          for (Node neighbor : this->instance_.graph().movement().get_neighbors(
                   pi->at(next_agt)))
          {
            // std::cout << "Setting agent " << next_agt << " to node " << neighbor << "\n";
            std::shared_ptr<Configuration> next = std::make_shared<Configuration>();
            for (Agent agt = 0; agt < next_agt; agt++)
              next->PushBack(a->at(agt));
            next->PushBack(neighbor);
            double g = g_local[a];
            if (neighbor != pi->at(next_agt))
            {
              g += 1;
            }
            g_local[next] = g;

            Configuration completed_next(*next);
            for(Agent agt = next->size(); agt < pi->size(); agt++){
              completed_next.PushBack(pi->at(agt));
            }
            open_local.insert(next, g, this->shortestPathHeuristics_.getHeuristic(completed_next));
            // std::cout << "\tAdded next with " << next->size() << " agents, total cost: " << g+ this->shortestPathHeuristics_.getHeuristic(*next) << "\n";
            // std::cout << "Next: ";
            // std::cout << *next;
            // std::cout << "\n";
          }
        }
      }
      return nullptr;
    }

    bool IsGoal(const std::shared_ptr<Configuration> &config)
    {
      if (config->size() != this->instance_.nb_agents())
        return false;
      for (size_t agt = 0; agt < config->size(); agt++)
        if (config->at(agt) != this->instance_.goal()[agt])
          return false;
      return true;
    }

  public:
    DFS(const Instance<GraphMove, GraphComm> &instance,
        const Objective &objective)
        : Solver<GraphMove, GraphComm>(instance, objective), exec_(), shortestPathHeuristics_(instance)
    {
      std::shared_ptr<Configuration> start = std::make_shared<Configuration>();
      for (size_t agt = 0; agt < instance.start().size(); agt++)
        start->PushBack(instance.start()[agt]);
      exec_.push_back(start);
      closed_.insert(start);
    }

    /**
     * @brief execute one step of computation of DFS from [Tateo et al.]
     *
     * @return true if the work is finished (we then found a solution)
     * @return false if the work is not finished
     */
    bool StepCompute() override
    {
      if (exec_.empty())
        return true;
      if (IsGoal(exec_.back()))
      {
        for (size_t agt = 0; agt < this->instance_.nb_agents(); agt++)
        {
          std::shared_ptr<Path> p_agt = std::make_shared<Path>();
          for (size_t t = 0; t < exec_.size(); t++)
          {
            p_agt->PushBack(exec_[t]->at(agt));
          }
          this->execution_.set_path(agt, p_agt);
        }
        return true;
      }

      std::shared_ptr<Configuration> config = FindBestConfiguration(exec_.back());

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
    /**
     * @brief compute an execution that goes
     * step steps towards the goal (but not necessary reaching the goal), and return the last configuration of this execution.
     * If the greedy algorithm is stuck, the function returns the last configuration that was generated.
     *
     * @return last configuration of the computed execution.
     */
    Configuration smallStepCompute(int step)
    {
      for (int i = 0; i < step; i++)
      {
        if (exec_.empty())
          return this->execution().get_configuration(this->execution().size());
        if (IsGoal(exec_.back()))
        {
          for (size_t agt = 0; agt < this->instance_.nb_agents(); agt++)
          {
            std::shared_ptr<Path> p_agt = std::make_shared<Path>();
            for (size_t t = 0; t < exec_.size(); t++)
            {
              p_agt->PushBack(exec_[t]->at(agt));
            }
            this->execution_.set_path(agt, p_agt);
          }
          break;
        }

        std::shared_ptr<Configuration> config = FindBestConfiguration(exec_.back());
        if (config == nullptr)
        {
          break;
        }
        else
        {
          closed_.insert(config);
          exec_.push_back(config);
        }
      }
      std::cout << "* Exec has size: " << exec_.size() << "\n";
      return *exec_.back();
    }
  };

} // namespace coupled
