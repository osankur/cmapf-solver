#pragma once

#define INFINITY std::numeric_limits<int>::max()

#include <Execution.hpp>
#include <Instance.hpp>
#include <Objective.hpp>
#include <Configuration.hpp>
#include <Solver.hpp>
#include <Logger.hpp>

namespace coordinated{
    enum class collision_mode_t {CHECK_COLLISIONS, IGNORE_COLLISIONS};

    template <class GraphMove, class GraphComm>
    class LocalQ {
        protected:
            // Full configuration for agents. The local Q function only depends on some of the components
            const Configuration source_conf_;
            
            const Instance<GraphMove,GraphComm> & instance_;
            FloydWarshall<GraphMove,GraphComm> & fw_;

            const collision_mode_t collision_mode_;
        public:
        
        bool check_collisionless(const Configuration & next_conf){
            std::set<Node> check;
            for(int i = 0; i < next_conf.size(); i++){
                auto node = next_conf[i];
                if (check.contains(node)) return false;
                check.insert(node);
            }
            return true;
        }

        bool check_connected(const Configuration & next_conf){
            std::set<Node> to_check;
            for(int i = 0; i < next_conf.size(); i++) to_check.insert(next_conf[i]);
            std::list<Node> open;
            open.push_back(next_conf[0]);
            while(!open.empty()){
                Node node = open.back();
                open.pop_back();
                to_check.erase(node);
                //LOG_INFO("Popped " + std::to_string(node) + ". Remaining " + std::to_string(to_check.size()) + " in open");
                auto neighbors = instance_.graph().communication().get_neighbors(node);
                // LOG_INFO("Neighborhood is: ");
                // for(auto node : neighbors){
                //     std::cout << node << ", ";
                // }
                // std::cout << "\n";
                for(auto m : to_check){
                    if (neighbors.contains(m)){
                        open.push_back(m);
                        // LOG_INFO("Adding " + std::to_string(m));
                    }
                }
            }
            if (to_check.empty()) return true;
            return false;
        }

        public:
        LocalQ(const Instance<GraphMove,GraphComm> & instance,
                FloydWarshall<GraphMove,GraphComm> & fw,
               const Configuration & source_conf, 
               const collision_mode_t collision_mode = collision_mode_t::CHECK_COLLISIONS)
            : source_conf_(source_conf), instance_(instance), fw_(fw) {
            }

        /** @pre next_partial_conf is a successor configuration for the agents in args
         *  @return 
         */
        virtual size_t get_value(const std::vector<Node> & next_partial_conf) = 0;

        /** @pre next_partial_conf contains an entry for each element of arguments()
         */
        size_t get_value(const std::map<Agent,Node> & next_partial_conf){
            std::vector<Node> next_partial_conf_v;
            for(auto agt : this->arguments()){
                next_partial_conf_v.push_back(next_partial_conf[agt]);
            }
            return get_value(next_partial_conf_v,fw_);
        }

        virtual std::vector<Agent> arguments() = 0;

        const Configuration & source_configuration(){
            return source_conf_;
        }
    };

    /** 
     * Original local Q functions such that Q_i(source_conf, next_conf) returns the Q-value for agent agent.
     * This returns infty if next_conf is not connected or has collisions, and the remaining distance 
     * to the target of Agent i from next_conf otherwise.
     */
    template <class GraphMove, class GraphComm>
    class LocalQIdentity : public LocalQ<GraphMove,GraphComm> {
        private:
        const Agent agent_;
        // The vector of agents on which the local Q function depends
        const std::vector<Agent> args_;

        public:
        LocalQIdentity(const Instance<GraphMove,GraphComm> & instance,
               FloydWarshall<GraphMove,GraphComm> & fw,
               const Configuration & source_conf, 
               const std::vector<Agent> & args,
               Agent agent, 
               const collision_mode_t collision_mode = collision_mode_t::CHECK_COLLISIONS)
            : LocalQ<GraphMove,GraphComm>(instance,fw,source_conf,collision_mode), agent_(agent), args_(args){
            }
        
        size_t get_value(const std::vector<Node> & next_partial_conf){
            assert(this->args_.size() <= this->instance.start().size());

            if (!this->check_connected(next_partial_conf)){
                return INFINITY;
            } else{
                return this->fw.ComputeShortestPathSize(this->next_conf[agent_], this->instance.goal()[agent_]);
            }
        }

        const std::vector<Agent> & arguments(){
            return args_;
        }
    };

    template <class GraphMove, class GraphComm>
    class LocalQCompound : public LocalQ<GraphMove,GraphComm>{
        private:
        // Map argument of the Q function to a sorted vector of heuristic value and successor node
        std::map<std::vector<Node>, std::vector<std::pair<size_t,Node> > > data_;
        
        std::vector<Agent> args_;

        // Vector of functions whose sum is represented in this compound
        std::vector<std::shared_ptr<LocalQ<GraphMove,GraphComm> > > qfuncs_;

        const Agent agent_to_eliminate_;

        /**
         * @pre neighbors is a vector neighbors of all nodes except for that of agent_to_eliminate
         * @pre next_partial_conf is an empty vector
         * @pre index is 0 
         */
        void fill_data(
                        std::vector<std::unordered_set<Node>> & neighbors,
                        std::vector<Node> & next_partial_conf,
                        std::map<Agent,Node> & next_partial_conf_m,
                        int index)
            {
                if (index < neighbors.size()){
                    for(auto next_node : neighbors[index]){
                        next_partial_conf.push_back(next_node);
                        next_partial_conf_m[args_[index]] = next_node;
                        fill_data(neighbors, next_partial_conf, next_partial_conf_m, index+1);
                        next_partial_conf.pop_back();
                    }
                } else {
                    std::vector<std::pair<size_t,Node> > image;
                    auto pivot_neighbors = this->instance_.graph().movement().neighbors(agent_to_eliminate_);
                    for (auto next_node : pivot_neighbors){
                        next_partial_conf_m[agent_to_eliminate_] = next_node;
                        size_t sum = 0;
                        for(auto qf : qfuncs_){
                            size_t qf_value = qf.get_value(next_partial_conf_m);
                            if (qf_value == INFINITY){
                                sum = INFINITY;
                                break;
                            }
                            sum += qf_value;
                        }
                        if (sum != INFINITY){
                            image.push_back(std::make_pair(sum,next_node));
                        }
                    }
                    std::sort(image.begin(),image.end());
                    data_[next_partial_conf] = image;
                }
                
            }
            void initialize(){
                // args are the union of the agents in all qfuncs \ {agent_to_eliminate_}
                std::set<Agent> all_agents;
                for(auto qf : qfuncs_){
                    for (auto agent : qf.arguments()){
                        all_agents.insert(agent);
                    }
                }
                all_agents.erase(agent_to_eliminate_);
                args_ = all_agents;

                // neighbors[i] is the movement neighborhood of the node of the agent args[i]
                std::vector<std::unordered_set<Node> > neighbors;
                for(auto agent : args_){
                    neighbors.push_back(this->instance_.graph().movement().neighbors(this->source_conf[agent]));                    
                }

                std::vector<Node> next_partial_conf;
                std::map<Agent,Node> next_partial_conf_m;
                fill_data(neighbors,next_partial_conf,next_partial_conf_m,0);
            }
        public:
        LocalQCompound(const Instance<GraphMove,GraphComm> & instance,
               FloydWarshall<GraphMove,GraphComm> & fw,
               const Configuration & source_conf, 
               const std::vector<std::shared_ptr<LocalQ<GraphMove,GraphComm> > > & qfuncs,
               const Agent agent_to_eliminate, 
               const collision_mode_t collision_mode = collision_mode_t::CHECK_COLLISIONS)
            : LocalQ<GraphMove,GraphComm>(instance,fw,source_conf,collision_mode), 
                qfuncs_(qfuncs), 
                agent_to_eliminate_(agent_to_eliminate)
            {
                initialize();
            }
        
        size_t get_value(const std::vector<Node> & next_partial_conf){
            const std::vector<std::pair<size_t,Node> > & values = get_value_vector(next_partial_conf);
            if (values.size() == 0){
                return INFINITY;
            } else{
                return values[0].first;
            }
        }

        /**
         * @pre next_partial_conf[i] is a movement-neighbor of the node of arguments()[i] in source_configuration()
         */
        const std::vector<std::pair<size_t,Node> > & get_value_vector(const std::vector<Node> & next_partial_conf){
            return data_[next_partial_conf];
        }
        /**
         * @pre next_partial_conf_m contains a key agent for each agent in arguments(), and next_partial_conf_m[agent] is 
         * a movement-neighbor of the node of in source_configuration()[agent]
         */
        const std::vector<std::pair<size_t,Node> > & get_value_vector(const std::map<Agent,Node> & next_partial_conf_m){
            std::vector<Node> next_partial_conf;
            for(Agent agent : arguments()){
                next_partial_conf.push_back(next_partial_conf_m.find(agent)->second);
            }
            return get_value_vector(next_partial_conf);
        }
        const std::vector<Agent> & arguments(){
            return args_;
        }
        Agent agent_to_eliminate(){
            return agent_to_eliminate_;
        }
        std::vector<std::shared_ptr<LocalQ<GraphMove,GraphComm> > > & qfunctions(){
            return qfuncs_;
        }
    };

    template <class GraphMove, class GraphComm>
    class CoordSolver : public Solver<GraphMove,GraphComm> {
    private:
        std::vector<std::shared_ptr<LocalQ<GraphMove,GraphComm> > > qfuncs_;
        unsigned int lookback_;
        unsigned int lookahead_;
        FloydWarshall<GraphMove,GraphComm> fw;
        const collision_mode_t collision_mode_;

        void initialize_qfuncs(const Configuration & source_conf){
            size_t nb_agents = this->instance_.nb_agents();
            for(size_t i = 0; i < nb_agents; i++){
                // Create Q function with arguments [max(i - lookback_,0),...,min(i+lookahead_,nb_agents-1)]
                std::vector<Agent> args;
                size_t window_l = (i - lookback_ >= 0) ? (i - lookback_) : 0;
                size_t window_r = (i + lookahead_ < nb_agents) ? i + lookahead_ : nb_agents -1;
                for(size_t j = window_l; j <= window_r; j++)
                    args.push_back(j);
                std::shared_ptr<LocalQIdentity<GraphMove,GraphComm>> qf = std::make_shared<LocalQIdentity>(this->instance_,fw,source_conf,args,i,collision_mode_);
            }
        }

        /**
         * @pre initialize_qfuncs has been called
         */
        void reduce_qfuncs(){

        }

        /**
         * @pre reduce_qfuncs has been called.
         */
        Configuration get_next_best(){

        }
    public:
    CoordSolver(const Instance<GraphMove, GraphComm>& instance, 
                const Objective& objective,
                unsigned int lookback,
                unsigned int lookahead,
                const collision_mode_t collision_mode = collision_mode_t::CHECK_COLLISIONS
                )
      : Solver<GraphMove, GraphComm>(instance, objective), collision_mode_(collision_mode), lookback_(lookback), lookahead_(lookahead),fw(instance) {}

    virtual bool StepCompute() {
        this->execution_ = Execution();
        /*
        auto graph = this->instance_.graph().movement();
        std::cerr << "Node count: " << graph.node_count() << "\n";
        const Configuration & start = this->instance_.start();
        const Configuration & goal = this->instance_.goal();
        std::cerr << "start has size: " << start.size() << "\n";
        std::cerr << "goal has size: " << goal.size() << "\n";
        for (int i = 0; i < start.size(); i++){
            std::cerr << "(" << start[i] << " -- ";
            std::cerr << goal[i] << "), ";        }
        std::cerr <<"\n";
        const std::vector<Agent> args ={0,1,2};
        FloydWarshall<GraphMove,GraphComm> fw(this->instance_);
        LocalQ<GraphMove,GraphComm> localQ(start, args, fw, this->instance_);
        assert(localQ.check_collisionless(start));
        assert(localQ.check_collisionless(goal));
        assert(localQ.check_connected(start));
        assert(localQ.check_connected(goal));
        localQ.get_value(start);
        */
        return true;
    }
    };
}