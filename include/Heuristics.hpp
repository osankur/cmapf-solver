#pragma once
#include <ShortestPathCalculator.hpp>
#include <Instance.hpp>
#include <Configuration.hpp>
#include <memory>

template <class GraphMove, class GraphComm>
class Heuristics
{
public:
    Heuristics() {}
    virtual double getHeuristic(const Configuration &c, const Configuration &goal) = 0;
    virtual double getHeuristic(const Node &n, const Node &goal) = 0;
    virtual int getBirdEyeHeuristic(const Configuration &c, const Configuration &goal) = 0;
    virtual int getShortestPathHeuristic(const Configuration &c, const Configuration &goal) = 0;
    virtual int getShortestPathDistance(const Node & c, const Node & goal, bool fullSingleSource=false) = 0;
    virtual int getBirdEyeDistance(const Node & c, const Node & goal) = 0;
};

template <class GraphMove, class GraphComm>
class ShortestPathHeuristics : public Heuristics<GraphMove, GraphComm>
{
private:
    double getL2Distance(std::pair<int, int> &p1, std::pair<int, int> &p2)
    {
        return sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
    }

    double getL1Distance(std::pair<int, int> &p1, std::pair<int, int> &p2)
    {
        return abs(p1.first - p2.first) + abs(p1.second - p2.second);
    }
public:
    ShortestPathHeuristics(const Instance<GraphMove, GraphComm> &instance) : instance_(instance), sp_(DijkstraSPCalculator<GraphMove, GraphComm>(instance)) {}
    ShortestPathHeuristics(const Instance<GraphMove, GraphComm> &instance, ShortestPathCalculator<GraphMove, GraphComm> & sp) : instance_(instance), sp_(sp) {}


    int getBirdEyeHeuristic(const Configuration &c, const Configuration &goal){
            double d = 0;
            for (int agt = 0; agt < c.size(); agt++)
            {
                d += this->getBirdEyeDistance(c.at(agt), goal.at(agt));
            }
            return d;

    }
    int getShortestPathHeuristic(const Configuration &c, const Configuration &goal){
        double d = 0;
        for (int agt = 0; agt < c.size(); agt++)
        {
            d += this->getShortestPathDistance(c.at(agt), goal.at(agt));
        }
        return d;
    }

    int getShortestPathDistance(const Node & c, const Node & goal, bool fullSingleSource=false) override {
        int d = this->sp_.getShortestPathDistance(c, goal, fullSingleSource);
        //std::cerr << "*** SP(" << c << ", " << goal << ") = " << d << "\n";
        return d;
    }
    int getBirdEyeDistance(const Node & c, const Node & goal) override{
        auto cpos = this->instance_.graph().movement().getPosition(c);
        auto gpos = this->instance_.graph().movement().getPosition(goal);
        auto d = this->getL1Distance(cpos, gpos);
        return this->getL1Distance(cpos, gpos);
    }

    double getHeuristic(const Configuration &c, const Configuration &goal) override
    {
        return this->getShortestPathHeuristic(c,goal);
    }
    double getHeuristic(const Node &c, const Node &goal) override
    {
        return this->getShortestPathDistance(c, goal);
    }

protected:
    ShortestPathCalculator<GraphMove, GraphComm> & sp_;
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
    BirdEyeHeuristics(const Instance<GraphMove, GraphComm> &instance, ShortestPathCalculator<GraphMove, GraphComm> & sp) : ShortestPathHeuristics<GraphMove, GraphComm>(instance, sp) {
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
            return this->getBirdEyeHeuristic(c, goal);
        }
    }
    double getHeuristic(const Node &c, const Node &goal) override
    {
        if (targetNodes.find(goal) != targetNodes.end())
        {
            return this->getShortestPathDistance(c, goal);
        }
        else
        {
            return this->getBirdEyeDistance(c, goal);
        }
    }
};
