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

    class HeapComparator
    {
    private:
      const Instance<GraphMove, GraphComm> &instance_;
      const std::shared_ptr<Configuration> config_;
      FloydWarshall<GraphMove, GraphComm> floydwarshall_;

    public:
      explicit HeapComparator(const Instance<GraphMove, GraphComm> &instance,
                              const std::shared_ptr<Configuration> config)
          : instance_(instance), config_(config), floydwarshall_(instance)
      {
        floydwarshall_.computeAllPairs();
      }

      bool operator()(const std::shared_ptr<Configuration> &a,
                      const std::shared_ptr<Configuration> &b)
      {
        size_t costA = 0;
        size_t costB = 0;
        costA += a->size();
        costB += b->size();
        for (size_t agt = 0; agt < config_->size(); agt++)
        {
          if (agt < a->size())
            costA += 1 + floydwarshall_.getShortestPathDistance(
                             a->at(agt), instance_.goal()[agt]);
          /*instance_.graph().movement().get_distance(config_->at(agt),
          a->at(agt)) + instance_.graph().movement().get_distance(a->at(agt),
          instance_.goal()[agt]);*/
          else
            costA += floydwarshall_.getShortestPathDistance(config_->at(agt),
                                                            instance_.goal()[agt]);
          // instance_.graph().movement().get_distance(config_->at(agt),
          // instance_.goal()[agt]);
          if (agt < b->size())
            costB += 1 + floydwarshall_.getShortestPathDistance(
                             b->at(agt), instance_.goal()[agt]);
          /*instance_.graph().movement().get_distance(config_->at(agt),
          b->at(agt)) + instance_.graph().movement().get_distance(b->at(agt),
          instance_.goal()[agt]);*/
          else
            costB += floydwarshall_.getShortestPathDistance(config_->at(agt),
                                                            instance_.goal()[agt]);
          // instance_.graph().movement().get_distance(config_->at(agt),
          // instance_.goal()[agt]);
        }
        return costA > costB;
      }
    };

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

      g_local[pi] = 0;

      open_local.insert(pi, 0, shortestPathHeuristics_.getHeuristic(*pi));
      int max_size = -1;
      while (!open_local.empty())
      {
        if (open_local.size() > max_size)
        {
          max_size = open_local.size();
        }
        auto a = open_local.pop();

        if (a->size() == this->instance_.nb_agents()
            && this->instance_.graph().communication().is_configuration_connected(*a) 
            && (closed_.find(a) == closed_.end()) && !((*a) == (*pi)))
        {
          std::cout << "open.size() == " << open_local.size() << " closed.size() == " << closed_.size() << "     ";
          std::cout.flush();
          return a;
        } else if (a->size() < pi->size()) {
          Agent next_agt = (Agent)a->size();
          for (Node neighbor : this->instance_.graph().movement().get_neighbors(
                  pi->at(next_agt)))
          {
            std::shared_ptr<Configuration> next = std::make_shared<Configuration>();
            for (Agent agt = 0; agt < next_agt; agt++)
              next->PushBack(a->at(agt));
            next->PushBack(neighbor);
            double g = g_local[a] + 1;
            g_local[next] = g;
            open_local.insert(next, g, this->shortestPathHeuristics_.getHeuristic(*next));
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

    std::shared_ptr<Configuration> FindBestChild(
        const std::shared_ptr<Configuration> config)
    {
      HeapComparator cmp(this->instance_, config);
      std::priority_queue<std::shared_ptr<Configuration>,
                          std::vector<std::shared_ptr<Configuration>>,
                          HeapComparator>
          open(cmp);
      std::cout << "\tRunning findBestChild:\n";
      std::cout.flush();
      std::shared_ptr<Configuration> start = std::make_shared<Configuration>();
      open.push(start);
      while (!open.empty())
      {
        std::cout << "\ropen.size() == " << open.size() << " closed.size() == " << closed_.size() << "     ";
        std::cout.flush();
        std::shared_ptr<Configuration> current = open.top();
        open.pop();

        if (current->size() == this->instance_.nb_agents())
        {
          if (this->instance_.graph().communication().is_configuration_connected(*current) &&
              closed_.find(current) == closed_.end())
          {
            return current;
          }
          else
            continue;
        }

        for (Node neighbor : this->instance_.graph().movement().get_neighbors(
                 config->at(current->size())))
        {
          std::shared_ptr<Configuration> next = std::make_shared<Configuration>();
          for (Agent agt = 0; agt < static_cast<Agent>(current->size()); agt++)
            next->PushBack(current->at(agt));
          next->PushBack(neighbor);
          open.push(next);
        }
      }
      std::cout << "\n";
      return nullptr;
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

        std::shared_ptr<Configuration> config = FindBestChild(exec_.back());
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
