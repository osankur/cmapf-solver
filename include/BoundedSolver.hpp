#pragma once
#include <Instance.hpp>
#include <Objective.hpp>
#include <Solver.hpp>
#include <Heuristics.hpp>
#include <ShortestPathCalculator.hpp>
#include <Path.hpp>
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

template <class GraphMove, class GraphComm>
class BoundedSolver
{
public:
    BoundedSolver<GraphMove, GraphComm>() = default;
    /**
     * @brief compute an execution that goes
     * `steps` steps from source towards goal (but not necessary reaching the goal), and return the last configuration of this execution.
     * @return An execution of length at most steps towards goal.
     */
    virtual std::vector<std::shared_ptr<Configuration>> computeBoundedPathTowards(const Configuration &source, const Configuration &goal, int steps) = 0;
};

namespace decoupled
{
    template <class GraphMove, class GraphComm>
    class BoundedDecoupledSolver : public BoundedSolver<GraphMove, GraphComm>
    {
    private:
        const Instance<GraphMove, GraphComm> &instance_;
        const Objective &objective_;
        std::shared_ptr<DijkstraSPCalculator<GraphMove, GraphComm>> sp_;

    public:
        BoundedDecoupledSolver(const Instance<GraphMove, GraphComm> &instance,
                               const Objective &objective,
                               std::shared_ptr<DijkstraSPCalculator<GraphMove, GraphComm>> floydwarshall) : instance_(instance), objective_(objective), sp_(floydwarshall)
        {
        }
        BoundedDecoupledSolver(const Instance<GraphMove, GraphComm> &instance,
                               const Objective &objective) : instance_(instance), objective_(objective), sp_(std::make_shared<DijkstraSPCalculator<GraphMove, GraphComm>>(instance))
        {
        }

        std::vector<std::shared_ptr<Configuration>> computeBoundedPathTowards(const Configuration &source, const Configuration &goal, int steps) override
        {
            std::vector<Path> spaths;
            for (int agt = 0; agt < this->instance_.nb_agents(); agt++)
            {
                spaths.push_back(this->sp_->getShortestPath(source.at(agt), goal.at(agt)));
            }
            std::vector<std::shared_ptr<Configuration>> segment;
            for (int i = 0; i < steps; i++)
            {
                Configuration c;
                for (int agt = 0; agt < this->instance_.nb_agents(); agt++)
                {
                    c.PushBack(spaths[agt].GetAtTimeOrLast(i));
                }
                if (instance_.graph().communication().isConfigurationConnected(c))
                {
                    segment.push_back(std::make_shared<Configuration>(c));
                }
            }
            return segment;
        }
    };
};