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
    // BoundedSolver<GraphMove, GraphComm>() {};
    /**
     * @brief compute an execution that goes
     * `steps` steps from source towards goal (but not necessary reaching the goal), and return the last configuration of this execution.
     * @return An execution of length at most steps towards goal.
     */
    virtual std::vector<std::shared_ptr<Configuration>> computeBoundedPathTowards(const Configuration &source, const Configuration &goal, int steps, int max_iterations = -1) = 0;
};
