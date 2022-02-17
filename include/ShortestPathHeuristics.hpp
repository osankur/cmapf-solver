#pragma once
#include <FloydWarshall.hpp>
#include <Instance.hpp>
#include <Configuration.hpp>

namespace coupled
{
    template <class GraphMove, class GraphComm>
    class ShortestPathHeuristics
    {
    public:
        ShortestPathHeuristics(const Instance<GraphMove,GraphComm> &instance) : instance_(instance), floydwarshall_(instance) {}
        int getHeuristic(const Configuration & c){
            const Configuration & goal = instance_.goal();
            int d = 0;
            for(int agt = 0; agt < c.size(); agt++){
                d += floydwarshall_.getShortestPathDistance(c.at(agt), goal.at(agt));
            }
            return d;
        }
    private:
        FloydWarshall<GraphMove, GraphComm> floydwarshall_;
        const Instance<GraphMove,GraphComm> & instance_;
    };

};