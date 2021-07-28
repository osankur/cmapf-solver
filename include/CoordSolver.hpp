#pragma once

#include <Execution.hpp>
#include <Instance.hpp>
#include <Objective.hpp>
#include <Configuration.hpp>

namespace coordinated{

    /**
     * TODO Test all functions
     * TODO Determine what FloydWarshall actually does
     */
    template <class GraphMove, class GraphComm>
    class LocalQ {
        protected:
            // Source configuration from which local Q function is to be computed
            const Configuration source_conf;
            // The vector of agents on which the local Q function depends
            const std::vector<Agent> & args;
            const FloydWarshall<GraphMove,GraphComm> & fw;
            const Instance<GraphMove,GraphComm> & instance;

        bool check_collisionless(const std::vector<Node> & next_conf){
            std::set<Node> check;
            for(auto node : next_conf){
                if (check.contains(node)) return false;
                check.insert(node);
            }
            return true;
        }

        bool check_connected(const std::vector<Node> & next_conf){
            std::set<Node> to_check;
            for(auto node : next_conf) to_check.insert(node);
            
            std::list<Node> open;
            open.push_back(next_conf[0]);
            while(!open.empty()){
                Node node = open.back();
                open.pop_back();
                to_check.erase(node);
                auto neighbors = instance.graph().movement().get_neighbors(node);
                for(auto m : to_check){
                    if (neighbors.contains(m)){
                        open.push_back(m);
                    }
                }
            }
            if (to_check.empty()) return true;
            return false;
        }


        public:
        LocalQ(const Configuration & source_conf, const std::vector<Agent> & args, 
               const FloydWarshall<GraphMove,GraphComm> & fw, const Instance<GraphMove,GraphComm> & instance)
            : source_conf(source_conf), args(args), fw(fw), instance(instance){
            }


        int get_value(const std::vector<Node> & next_conf){
            assert(next_conf.size() == args.size());
            if (!check_connected(next_conf))
                std::numeric_limits<int>::max();
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