#pragma once
#include <FloydWarshall.hpp>
#include <Instance.hpp>
#include <Configuration.hpp>
#include <memory>

float getEuclidianDistance(std::pair<int, int> &p1, std::pair<int, int> &p2)
{
    return sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
}

float getL1Distance(std::pair<int, int> &p1, std::pair<int, int> &p2)
{
    return abs(p1.first - p2.first) + abs(p1.second - p2.second);
}

template <class GraphMove, class GraphComm>
class Heuristics
{
public:
    Heuristics() {}
    virtual double getHeuristic(const Configuration &c, const Configuration &goal) = 0;
    virtual double getHeuristic(const Node &n, const Node &goal) = 0;
    virtual std::shared_ptr<FloydWarshall<GraphMove, GraphComm>> getFloyd() = 0;
};

template <class GraphMove, class GraphComm>
class ShortestPathHeuristics : public Heuristics<GraphMove, GraphComm>
{
public:
    ShortestPathHeuristics(const Instance<GraphMove, GraphComm> &instance) : instance_(instance), floydwarshall_(std::make_shared<FloydWarshall<GraphMove, GraphComm>>(instance)) {}
    ShortestPathHeuristics(const Instance<GraphMove, GraphComm> &instance, std::shared_ptr<FloydWarshall<GraphMove, GraphComm>> floydwarshall) : instance_(instance), floydwarshall_(floydwarshall) {}
    std::shared_ptr<FloydWarshall<GraphMove, GraphComm>> getFloyd()
    {
        return this->floydwarshall_;
    }

    double getHeuristic(const Configuration &c, const Configuration &goal) override
    {
        double d = 0;
        for (int agt = 0; agt < c.size(); agt++)
        {
            d += floydwarshall_->getShortestPathDistance(c.at(agt), goal.at(agt));
        }
        return d;
    }
    double getHeuristic(const Node &c, const Node &goal) override
    {
        return this->floydwarshall_->getShortestPathDistance(c, goal);
    }

protected:
    std::shared_ptr<FloydWarshall<GraphMove, GraphComm>> floydwarshall_;
    const Instance<GraphMove, GraphComm> &instance_;
};

template <class GraphMove, class GraphComm>
class BirdEyeHeuristics : public ShortestPathHeuristics<GraphMove, GraphComm>
{
private:
    std::set<Agent> targetNodes;
    void init(){
        for(auto n : this->instance_.goal()){
            targetNodes.insert(n);
        }
    }
public:
    BirdEyeHeuristics(const Instance<GraphMove, GraphComm> &instance) : ShortestPathHeuristics<GraphMove, GraphComm>(instance) {
        init();
    }
    BirdEyeHeuristics(const Instance<GraphMove, GraphComm> &instance, std::shared_ptr<FloydWarshall<GraphMove, GraphComm>> fw) : ShortestPathHeuristics<GraphMove, GraphComm>(instance, fw) {
        init();
    }
    double getHeuristic(const Configuration &c, const Configuration &goal) override
    {
        if (goal == this->instance_.goal())
        {
            return ShortestPathHeuristics<GraphMove, GraphComm>::getHeuristic(c, goal);
        }
        else
        {
            double d = 0;
            for (int agt = 0; agt < c.size(); agt++)
            {
                auto cpos = this->instance_.graph().movement().get_position(c.at(agt));
                auto gpos = this->instance_.graph().movement().get_position(goal.at(agt));
                d += getL1Distance(cpos, gpos);
            }
            return d;
        }
    }
    double getHeuristic(const Node &c, const Node &goal) override
    {
        if (targetNodes.find(goal) != targetNodes.end())
        {
            return this->floydwarshall_->getShortestPathDistance(c, goal);
        }
        else
        {
            auto cpos = this->instance_.graph().movement().get_position(c);
            auto gpos = this->instance_.graph().movement().get_position(goal);
            return getL1Distance(cpos, gpos);
        }
    }
};
