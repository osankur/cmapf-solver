#pragma once
#include <FloydWarshall.hpp>
#include <Instance.hpp>
#include <Configuration.hpp>

namespace coupled
{

    float getEuclidianDistance(std::pair<int,int> & p1, std::pair<int,int> & p2){
        return sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
    }

    float getL1Distance(std::pair<int,int> & p1, std::pair<int,int> & p2){
        return abs(p1.first - p2.first) + abs(p1.second - p2.second);
    }

    template <class GraphMove, class GraphComm>
    class Heuristics
    {
        public:
        Heuristics(){}
        virtual double getHeuristic(const Configuration & c, const Configuration & goal) = 0;
    };

    template <class GraphMove, class GraphComm>
    class ShortestPathHeuristics : public Heuristics<GraphMove, GraphComm>
    {
    public:
        ShortestPathHeuristics(const Instance<GraphMove,GraphComm> &instance) : instance_(instance), floydwarshall_(instance) {}
        double getHeuristic(const Configuration & c, const Configuration & goal) override {
            double d = 0;
            for(int agt = 0; agt < c.size(); agt++){
                d += floydwarshall_.getShortestPathDistance(c.at(agt), goal.at(agt));
            }
            return d;
        }
    protected:
        FloydWarshall<GraphMove, GraphComm> floydwarshall_;
        const Instance<GraphMove,GraphComm> & instance_;
    };


    template <class GraphMove, class GraphComm>
    class BirdEyeHeuristics : public ShortestPathHeuristics<GraphMove, GraphComm>
    {
    public:
        BirdEyeHeuristics(const Instance<GraphMove,GraphComm> &instance) : ShortestPathHeuristics<GraphMove,GraphComm>(instance) {}
        double getHeuristic(const Configuration & c, const Configuration & goal) override {
            if ( goal == this->instance_.goal() ){
                return ShortestPathHeuristics<GraphMove, GraphComm>::getHeuristic(c, goal);
            } else {
            double d = 0;
            for(int agt = 0; agt < c.size(); agt++){
                auto cpos = this->instance_.graph().movement().get_position(c.at(agt));
                auto gpos = this->instance_.graph().movement().get_position(goal.at(agt));
                d += getL1Distance(cpos, gpos);
            }
            return d;
            }
        }
    };




};