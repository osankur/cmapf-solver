#pragma once

#include <Execution.hpp>
#include <Instance.hpp>
#include <Objective.hpp>
#include <Configuration.hpp>

namespace coordinated{

    template <class GraphMove, class GraphComm>
    class LocalQ {
        protected:
            // Source configuration from which local Q function is to be computed
            const Configuration source_conf;
            // The vector of agents on which the local Q function depends
            const std::vector<Agent> & args;
            std::shared_ptr<FloydWarshall<GraphMove,GraphComm>> fw;
        public:
        LocalQ(const Configuration & source_conf, const std::vector<Agent> & args, const Instance<GraphMove,GraphComm> & instance) 
            : source_conf(source_conf), args(args){
                fw = std::make_shared(instance);
            }

        int get_value(const std::vector<Node> & next_conf){
            assert(next_conf.size() == args.size());
            // Check if next_conf is connected or if it contains collisions
            //  --> std::numeric_limits<Node>::max()
            if (0) return std::numeric_limits<int>::max();
            int sum = 0;
            for(int i = 0; i < args.size(); i++){
                Agent agent = args[i];
                Node node = next_conf[i];
                sum += fw.getShortestPathSize(instance.goal()[i], node);
            }
            return sum;
        }
    };

    template <class GraphMove, class GraphComm>
    class CoordSolver {
    protected:
    const Instance<GraphMove, GraphComm>& instance_;
    const Objective& objective_;
    Execution execution_;

    public:
    CoordSolver(const Instance<GraphMove, GraphComm>& instance)
        : instance_(instance), execution_() {}

    const Execution Compute(){return Execution();}
    const Instance<GraphMove, GraphComm>& instance() const { return instance_; }
    const Objective& objective() const { return objective_; }
    };
}