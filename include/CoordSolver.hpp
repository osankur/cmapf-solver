/**
 * @file CoordSolver.hpp
 * 
 * @author Ocan Sankur <ocan.sankur@cnrs.fr>
 * @brief Greedy CMAPF algorithm with coordinated action selection mechanism, assuming window-connectivity of the configurations.
 * @date 2022-06-12
 * 
 * @copyright Copyright (c) 2022 Ocan Sankur
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

 //#define COORD_DEBUG

#define INFINITY std::numeric_limits<int>::max()

#include <CMAPF.hpp>
#include <Execution.hpp>
#include <Instance.hpp>
#include <Objective.hpp>
#include <Configuration.hpp>
#include <Solver.hpp>
#include <Logger.hpp>
#include <Heuristics.hpp>
#include <set>
#include <queue>
#include <unordered_set>

namespace coordinated
{

    /**
     * @brief An instance of LocalQ represents a Q-function for a given source and successor configuration,
     * that is, it is of type getSourceConfiguration() x Configuration -> size_t.
     * Moreover, this class fixes the first argument, and the function only depends on the positions of arguments() in the second argument.
     * That is, given a configuration c that is successor of getSourceConfiguration(), the value of the Q-function depends on c and agents that are in arguments().
     * LocalQIdentity implements a function that returns a shortest path distance to goal for a given agent
     *
     * @tparam GraphMove
     * @tparam GraphComm
     */
    template <class GraphMove, class GraphComm>
    class LocalQ
    {
    protected:
        // Fixed source configuration from which the Q function is to be computed
        const Configuration source_conf_;
        // Fixed goal configuration which determines the heuristic values used in the Q functions
        const Configuration goal_conf_;

        const Instance<GraphMove, GraphComm> &instance_;
        Heuristics<GraphMove, GraphComm> &heuristics_;

    public:
        bool isCollisionless(const std::vector<Node> &next_conf)
        {
            std::set<Node> check;
            for (int i = 0; i < next_conf.size(); i++)
            {
                auto node = next_conf[i];
                if (check.contains(node))
                    return false;
                check.insert(node);
            }
            return true;
        }

        bool isConnected(const std::vector<Node> &next_conf)
        {
            std::set<Node> to_check;
            for (int i = 0; i < next_conf.size(); i++)
                to_check.insert(next_conf[i]);
            std::list<Node> open;
            open.push_back(next_conf[0]);
            while (!open.empty())
            {
                Node node = open.back();
                open.pop_back();
                to_check.erase(node);
                // LOG_INFO("Popped " + std::to_string(node) + ". Remaining " + std::to_string(to_check.size()) + " in open");
                auto neighbors = instance_.graph().communication().get_neighbors(node);
                // LOG_INFO("Neighborhood is: ");
                // for(auto node : neighbors){
                //     std::cout << node << ", ";
                // }
                // std::cout << "\n";
                for (auto m : to_check)
                {
                    if (neighbors.contains(m))
                    {
                        open.push_back(m);
                        // LOG_INFO("Adding " + std::to_string(m));
                    }
                }
            }
            if (to_check.empty())
                return true;
            return false;
        }

    public:
        LocalQ(const Instance<GraphMove, GraphComm> &instance,
               Heuristics<GraphMove, GraphComm> &heuristics,
               const Configuration &source_conf,
               const Configuration &goal_conf)
            : source_conf_(source_conf), goal_conf_(goal_conf), instance_(instance), heuristics_(heuristics)
        {
        }

        /**
         * @brief Given a successor partial configuration which is defined for arguments(),
         * returns the value of the Q-function.
         *
         * @pre next_partial_conf contains an entry for each element of arguments()
         */
        virtual size_t getValue(const std::map<Agent, Node> &next_partial_conf) = 0;

        /**
         * @brief The list of agents on which the Q-function depends.
         *
         * @return const std::vector<Agent>&
         */
        virtual const std::vector<Agent> &arguments() = 0;

        const Configuration &getSourceConfiguration()
        {
            return source_conf_;
        }
        const Configuration &getGoalConfiguration()
        {
            return goal_conf_;
        }
    };

    /**
     * @brief Original local Q functions such that Q_i(source_conf, next_conf) returns the Q-value for agent i.
     * This returns infty if next_conf is not connected or has collisions, and the remaining distance
     * to the target of Agent i from next_conf otherwise.
     */
    template <class GraphMove, class GraphComm>
    class LocalQIdentity : public LocalQ<GraphMove, GraphComm>
    {
    private:
        const Agent agent_;
        // The vector of agents on which the local Q function depends
        const std::vector<Agent> args_;

    public:
        LocalQIdentity(const Instance<GraphMove, GraphComm> &instance,
                       Heuristics<GraphMove, GraphComm> &heuristics,
                       const Configuration &source_conf,
                       const Configuration &goal_conf,
                       const std::vector<Agent> &args,
                       Agent agent)
            : LocalQ<GraphMove, GraphComm>(instance, heuristics, source_conf, goal_conf), agent_(agent), args_(args)
        {
        }

        /**
         * @brief Return if next_partial_conf is disconnected or has collisions, return INFINITY; otherwise return the heuristic for agent this->agent_.
         *
         * @param next_partial_conf
         * @return size_t
         */
        size_t getValue(const std::map<Agent, Node> &next_partial_conf)
        {
            assert(this->args_.size() <= this->instance_.nb_agents());
            // std::cerr << "Computing value of the following next_partial_conf (size: "
            //           <<  next_partial_conf.size() << "): ";
            //  for(Node node : next_partial_conf){
            //      std::cerr << node << " ";
            //  }
            //  std::cerr << "\n";
            Configuration our_nodes;
            for (auto agent : this->arguments())
            {
                our_nodes.push_back(next_partial_conf.find(agent)->second);
            }
            // LOG_DEBUG("Are they connected: " + std::to_string(this->isConnected(our_nodes)));
            if (!this->isConnected(our_nodes))
            {
                return INFINITY;
            }
            // else if (this->instance_.getCollisionMode() == CollisionMode::CHECK_COLLISIONS && !this->isCollisionless(our_nodes))
            else if (this->instance_.getCollisionMode() == CollisionMode::CHECK_COLLISIONS && our_nodes.hasCollisions())
            {
                return INFINITY;
            }
            else
            {
                // auto d = this->heuristics_.getHeuristic(next_partial_conf.find(agent_)->second, this->instance_.goal()[agent_]);
                //  std::cout << "getValue called for " << next_partial_conf.find(agent_)->second << " to " <<
                //        this->instance_.goal()[agent_] << "\n";
                //  auto d1 = this->heuristics_.getFloyd()->getShortestPathDistance(next_partial_conf.find(agent_)->second,
                //       this->instance_.goal()[agent_]);

                size_t d = 0;

                // h: heuristic values from successors to goal
                d += (size_t)this->heuristics_.getHeuristic(next_partial_conf.find(agent_)->second, this->goal_conf_[agent_]);
                // std::cout << "\t\tAccessing heuristic " << next_partial_conf.find(agent_)->second << " to " <<this->goal_conf_[agent_] << " which is " << (size_t)this->heuristics_.getHeuristic(next_partial_conf.find(agent_)->second, this->goal_conf_[agent_]) << "\n";

#ifdef COORD_DEBUG

                std::cout << "\tQId_" << agent_ << "("; // \tsource " << this->getSourceConfiguration();
                Configuration nextc(this->getSourceConfiguration().size());
                for (auto n : next_partial_conf)
                {
                    // std::cout << "(Agent " << n.first << " @ " << n.second << "), ";
                    nextc[n.first] = n.second;
                }
                std::cout << nextc;
                // std::cout << "\n\tgoal " << this->goal_conf_;
                // std::cout << "\n\ti.e. distance bwt " << next_partial_conf.find(agent_)->second << " and " << this->goal_conf_[agent_] << " (+1)";
                std::cout << ") = " << d << "\n";
#endif
                // if (d1 != d2){
                //     std::cout << "FW = " << d1 << " but heuristics = " << d2 << "\n";
                //     std::cout << "source: " << next_partial_conf.find(agent_)->second;
                //     std::cout << " target: " << this->instance_.goal()[agent_];
                //     std::cout << "\n";
                // }
                // // LOG_DEBUG("Evaluating weight for node=" + std::to_string(agent_)
                //     + ". Distance from node " + std::to_string(next_partial_conf.find(agent_)->second)
                //     + " to node " + std::to_string(this->instance_.goal()[agent_]) + " = " + std::to_string(d));
                return d;
            }
        }

        const std::vector<Agent> &arguments()
        {
            return args_;
        }
    };

    /**
     * @brief Given a list of qfunctions defined on arguments(), and an agent_to_eliminate, this computes a function f that maps
     * a configuration for arguments() \ {agent_to_eliminate} to a sorted list of (Q-value, successor-node for agent_to_eliminate)
     * such that the first element of this list is the successor for agent_to_eliminate that minimizes the Q-value given the partial
     * successor configuration for arguments().
     *
     * @tparam GraphMove
     * @tparam GraphComm
     */
    template <class GraphMove, class GraphComm>
    class LocalQCompound : public LocalQ<GraphMove, GraphComm>
    {
    private:
        const Agent agent_to_eliminate_;

        // Map successor configuration for arguments() to a sorted vector of ((heuristic total cost 'g+h', minus g), successor node of agent_to_eliminate_)
        // Here g is the number of agents that have changed positions between source and successor. We want to minimize cost, and among these minimals, maximize g
        std::map<std::vector<Node>, std::vector<std::pair<std::pair<size_t,int>, Node>>> data_;

        std::vector<Agent> args_;

        // Vector of functions whose sum is represented in this compound
        std::vector<std::shared_ptr<LocalQ<GraphMove, GraphComm>>> qfuncs_;

        /** Enumerate all possible successor configurations for the agents arguments(),
         *  and associate to each such configuration a sorted vector of successors for
         *  agent_to_eliminate along with its heuristitic value. This is stored in data_.
         *
         * @pre neighbors is a vector neighbors of all nodes (parallel to args_) except for that of agent_to_eliminate
         * @pre next_partial_conf is an empty vector
         * @pre index is 0
         */
        void fill_data_rec(std::vector<std::unordered_set<Node>> &neighbors,
                           std::vector<Node> &next_partial_conf,
                           std::map<Agent, Node> &next_partial_conf_m,
                           int index)
        {
            if (index < neighbors.size())
            {
                for (auto next_node : neighbors[index])
                {
                    next_partial_conf.push_back(next_node);
                    next_partial_conf_m[args_[index]] = next_node;
                    fill_data_rec(neighbors, next_partial_conf, next_partial_conf_m, index + 1);
                    next_partial_conf.pop_back();
                }
            }
            else
            {
                // At this point, next_partial_conf contains a successor node for all agents in arguments(). (This does not include agent_to_eliminate_).
                // We are going to sort the values of the sum of qfuncs_ by quantifying over the successors for agent_to_eliminate_
                // FIXME remove the the g component 
                std::vector<std::pair<std::pair<size_t,int>, Node>> image;
                std::unordered_set<Node> pivot_neighbors(this->instance_.graph().movement().get_neighbors(this->source_conf_[agent_to_eliminate_]));
                pivot_neighbors.insert(this->source_conf_[agent_to_eliminate_]); // add the self-loop

#ifdef COORD_DEBUG
                std::cout << "Start configuration: " << this->getSourceConfiguration();
                std::cout << "\nAgent to eliminate: " << agent_to_eliminate_ << "\n";
                std::cout << "Arguments: [!" << agent_to_eliminate_ << "! ";
                for (auto agt : this->arguments()){
                    std::cout << agt << " ";
                }
                std::cout << "]\n";
                std::cout << "For next_partial_conf: < ";
                for (auto n : next_partial_conf_m)
                {
                    std::cout << "Agent " << n.first << " @ " << n.second << ", ";
                }
                std::cout << ">\n";
#endif
                for (auto next_node : pivot_neighbors)
                {
                    next_partial_conf_m[agent_to_eliminate_] = next_node;
#ifdef COORD_DEBUG
                    std::cout << "\tagent_to_eliminate at node: " << next_node << "\n";
#endif
                    size_t sum = 0;
                    for (auto qf : qfuncs_)
                    {
                        size_t qf_value = qf->getValue(next_partial_conf_m);
                        if (qf_value == INFINITY)
                        {
                            sum = INFINITY;
                            break;
                        }
                        sum += qf_value;
                    }

                    // size_t g = 0;
                    // for (size_t i = 0; i < args_.size(); i++){
                    //     if (next_partial_conf[args_[i]] != this->getSourceConfiguration()[args_[i]]){
                    //         g++;
                    //     }
                    // }
                    // if (next_node != this->getSourceConfiguration()[agent_to_eliminate_]){
                    //     g++;
                    // }
#ifdef COORD_DEBUG
                    std::cout << "\t\tdist = " << sum << "\n";
//                     std::cout << "\t\tg = " << g << "\n";
#endif
                    // sum += g;
                    if (sum != INFINITY)
                    {
                        image.push_back(std::make_pair(std::make_pair(sum,0), next_node));
                    }
                    next_partial_conf_m.erase(agent_to_eliminate_);
                }
                std::sort(image.begin(), image.end());

                // for(auto i : image){
                //     std::cout << "(node=" << i.second << ", x=" <<
                //     this->instance_.graph().movement().getPosition(i.second).first
                //     << ", y=" << this->instance_.graph().movement().getPosition(i.second).second
                //     << ", dist=" << i.first << ") ";
                // }
                // std::cout << "\n";

                data_[next_partial_conf] = image;
            }
        }
        void initialize()
        {
            // args are the union of the agents in all qfuncs \ {agent_to_eliminate_}
            std::set<Agent> all_agents;
            for (auto qf : qfuncs_)
            {
                for (auto agent : qf->arguments())
                {
                    all_agents.insert(agent);
                }
            }
            all_agents.erase(agent_to_eliminate_);
            args_ = std::vector(all_agents.begin(), all_agents.end());

            // neighbors[i] is the movement neighborhood of the node of the agent args_[i] from source_conf_[args_[i]]
            std::vector<std::unordered_set<Node>> neighbors;
            for (auto agent : args_)
            {
                std::unordered_set<Node> n(this->instance_.graph().movement().get_neighbors(this->source_conf_[agent]));
                n.insert(this->source_conf_[agent]); // add the self-loop
                neighbors.push_back(n);
            }

            // std::cerr << "Created local Q compound with args: ";
            // for(auto i : arguments()){
            //     std::cerr << i <<" ";
            // }
            // std::cerr << "\n";
            // Auxiliary fields for the recursive call to fill_data_rec
            std::vector<Node> next_partial_conf;
            std::map<Agent, Node> next_partial_conf_m;
            fill_data_rec(neighbors, next_partial_conf, next_partial_conf_m, 0);
#ifdef COORD_DEBUG
            for (auto p : data_){
                std::cout << "Data(<";
                for (int i = 0 ; i < this->arguments().size(); i++){
                    std::cout << "Agent " << this->arguments()[i] << " @ " << p.first[i] << " ";
                }
                std::cout << ">) ->\n";
                for (auto q : p.second){
                    std::cout << "\t Node: " << q.second << " with dist: " << q.first.first << " and g = " << q.first.second << "\n";
                } 
                
            }
#endif
        }

    public:
        LocalQCompound(const Instance<GraphMove, GraphComm> &instance,
                       Heuristics<GraphMove, GraphComm> &heuristics,
                       const Configuration &source_conf,
                       const Configuration &goal_conf,
                       const std::vector<std::shared_ptr<LocalQ<GraphMove, GraphComm>>> &qfuncs,
                       const Agent agent_to_eliminate)
            : LocalQ<GraphMove, GraphComm>(instance, heuristics, source_conf, goal_conf),
              qfuncs_(qfuncs),
              agent_to_eliminate_(agent_to_eliminate)
        {
            initialize();
        }

        size_t getValue(const std::map<Agent, Node> &next_partial_conf)
        {
            const std::vector<std::pair<std::pair<size_t,int>, Node>> &values = getValueVector(next_partial_conf);
            if (values.size() == 0)
            {
                return INFINITY;
            }
            else
            {
                // values[0].first.first is the cost
                // values[0].first.second is the -g value
                return values[0].first.first;
            }
        }

        /**
         * @pre next_partial_conf_m contains a key agent for each agent in arguments(), and next_partial_conf_m[agent] is
         * a movement-neighbor of the node of in getSourceConfiguration()[agent]
         */
        const std::vector<std::pair<std::pair<size_t,int>, Node>> &getValueVector(const std::map<Agent, Node> &next_partial_conf_m)
        {
            std::vector<Node> next_partial_conf;
            for (Agent agent : arguments())
            {
                next_partial_conf.push_back(next_partial_conf_m.find(agent)->second);
            }
            return data_[next_partial_conf];
        }
        const std::vector<Agent> &arguments()
        {
            return args_;
        }
        Agent agent_to_eliminate()
        {
            return agent_to_eliminate_;
        }
        std::vector<std::shared_ptr<LocalQ<GraphMove, GraphComm>>> &qfunctions()
        {
            return qfuncs_;
        }

        // void print()
        // {
        //     for (const auto p : data_)
        //     {
        //         const std::vector<Node> &key = p.first;
        //         const auto &val = p.second;
        //         std::cerr << "CONF ";
        //         for (auto n : key)
        //         {
        //             std::cerr << n << " ";
        //         }
        //         std::cerr << ": ";
        //         for (auto n : p.second)
        //         {
        //             std::cerr << "(node=" << n.second << ", dist=" << n.first << ") ";
        //         }
        //         std::cerr << "\n";
        //     }
        // }
    };

    template <class GraphMove, class GraphComm>
    class CoordSolver : public Solver<GraphMove, GraphComm>, public BoundedSolver<GraphMove, GraphComm>
    {
    private:
        int max_iterations = -1;
        int nb_iterations = 0;
        std::vector<std::shared_ptr<LocalQ<GraphMove, GraphComm>>> qfuncs_;
        unsigned int window_size_;
        Heuristics<GraphMove, GraphComm> &heuristics_;

        std::vector<Configuration> config_stack_;
        std::set<Configuration> closed_;

        std::map<std::pair<Configuration,Configuration>, std::vector<std::shared_ptr<Configuration>>> bounded_queries_;

        void initialize_qfuncs(const Configuration &source_conf, const Configuration &goal_conf)
        {
            qfuncs_.clear();
            size_t nb_agents = this->instance_.nb_agents();
            std::vector<std::vector<Agent>> clusters;
            std::vector<Agent> cl;
            int local_index = 0;
            for (size_t i = 0; i < nb_agents; i++)
            {
                cl.push_back(i);
                if (local_index == window_size_ - 1)
                {
                    clusters.push_back(cl);
                    cl.clear();
                    cl.push_back(i);
                    local_index = 0;
                }
                local_index++;
            }
            if (cl.size() >= 1)
            {
                clusters.push_back(cl);
            }
            for (int i = 0; i < nb_agents; i++)
            {
                int cluster_index = (i == 0) ? 0 : (i - 1) / (window_size_ - 1);
                auto cl = clusters[cluster_index];

                std::shared_ptr<LocalQIdentity<GraphMove, GraphComm>> qf =
                    std::make_shared<LocalQIdentity<GraphMove, GraphComm>>(this->instance_, heuristics_,
                                                                           source_conf, goal_conf, cl, i);
#ifdef COORD_DEBUG
                std::cout << "Creating Q function for agent " << i << " with args=[";
                for (auto j : cl)
                {
                    std::cout << j << " ";
                }
                std::cout << "]\n";
#endif
                qfuncs_.push_back(qf);
            }
        }

        /**
         * @brief remove qfunctions whose arguments contain agent_to_eliminate, and combine them in a LocalQCompound, which is added back
         * as a new qfunction
         *
         * @pre initialize_qfuncs has been called
         * @pre agent_to_eliminate appears in the arguments of some qfuncs
         */
        void reduce_qfuncs(Agent agent_to_eliminate)
        {
#ifdef COORD_DEBUG
                std::cout << "Eliminating agent " << agent_to_eliminate << "\n";
#endif
            // qfuncs to eliminate
            std::vector<std::shared_ptr<LocalQ<GraphMove, GraphComm>>> qfuncs_elim;
            // qfuncs that remain
            std::vector<std::shared_ptr<LocalQ<GraphMove, GraphComm>>> qfuncs_rem;
            for (auto qf : qfuncs_)
            {
                // if agent_to_eliminate is present in qf->arguments()
                if (std::find(qf->arguments().begin(), qf->arguments().end(), agent_to_eliminate) != qf->arguments().end())
                {
                    qfuncs_elim.push_back(qf);
                }
                else
                {
                    qfuncs_rem.push_back(qf);
                }
            }
            if (qfuncs_elim.size() == 0)
                return;
            std::shared_ptr<LocalQCompound<GraphMove, GraphComm>> qf_comp =
                std::make_shared<LocalQCompound<GraphMove, GraphComm>>(this->instance_, this->heuristics_,
                                                                       qfuncs_elim[0]->getSourceConfiguration(),
                                                                       qfuncs_elim[0]->getGoalConfiguration(),
                                                                       qfuncs_elim,
                                                                       agent_to_eliminate);
            // std::cout << "After eliminating " << agent_to_eliminate << "\n";
            // std::cout << "\tArguments of the compound: ";
            // for (auto a : qf_comp->arguments()){
            //     std::cout << a << " ";
            // }
            // std::cout << "\n";
            qfuncs_rem.push_back(qf_comp);
            qfuncs_ = qfuncs_rem;
        }


        class CoordNode {
            public:
            /**
             * @brief Construct a new Coord Node object
             * 
             * @param partial_conf 
             * @param g number of steps 
             * @param dist 
             */
            CoordNode(std::map<Agent, Node> &partial_conf, size_t dist, size_t g) : partial_conf(partial_conf), dist(dist), g(g){
            }
            size_t size() const {
                return partial_conf.size();
            }
            size_t getCost() const{
                return dist+g;
            }
            size_t getDistance() const{
                return dist;
            }
            size_t getG() const{
                return g;
            }
            std::map<Agent, Node> partial_conf;
            size_t dist;
            size_t g;
        };
        struct CoordNodeCmp
        {
            bool operator()(const CoordNode & a, const CoordNode & b) const
            {
                return (a.getCost() > b.getCost() ||
                        a.getCost() == b.getCost() && a.getG() < b.getG() ||
                        a.getCost() == b.getCost() && a.getG() == b.getG() && &a < &b);
            }
        };
        bool get_next_best_rec(const Configuration &source,
                               std::vector<std::shared_ptr<LocalQCompound<GraphMove, GraphComm>>> &linearized,
                               std::map<Agent, Node> &partial_conf)
        {
            size_t nb_agents = this->instance().nb_agents();
            std::map<Agent, Node> init_partial_conf;
            CoordNodeCmp cmp;
            std::priority_queue<CoordNode, std::vector<CoordNode>, CoordNodeCmp> open(cmp);
            CoordNode init_node(init_partial_conf,0,0);
            open.push(init_node);
            while(!open.empty()){

                this->nb_iterations++;
                if (this->max_iterations > 0 && this->nb_iterations > this->max_iterations){
                    return false;
                }
                // if (nb_iterations % 100 == 0)
                // std::cout << "\tnb_iterations:" << nb_iterations << "\n";

                CoordNode cnode = open.top();
                open.pop();
#ifdef COORD_DEBUG
                std::cout << "\nPopped: <";
                for (auto p : cnode.partial_conf)
                {
                    std::cout << "Agent " << p.first << " @ " << p.second << ", ";
                }
                std::cout << "> with cost: " << cnode.getCost() << " and g: " << cnode.getG() << "\n";
                std::cout << open.size() << " elements remain\n";
#endif
                if (cnode.size() == nb_agents){      
                    Configuration c;
                    for (Agent i = 0; i < this->instance_.nb_agents(); i++)
                    {
                        c.push_back(cnode.partial_conf[i]);
                    }
                    if (   closed_.find(c) != closed_.end()
                        || this->instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS && c.hasCollisions()
                        || !this->instance().graph().communication().isConfigurationConnected(c))
                    {
                        continue;
                    } else {
                        closed_.insert(c);
    #ifdef COORD_DEBUG
                        std::cout << ANSI_YELLOW << "Selected successor: " << c;
                        std::cout << " with g = " << cnode.g << ", cost = " << cnode.getCost() << "\n" << ANSI_RESET;
                        // std::cout << ANSI_YELLOW << "Selected successor: " << c;
                        // std::cout << "\n" << ANSI_RESET;
    #endif  
                        partial_conf = cnode.partial_conf;
                        if (nb_iterations > 10000) std::cout << "Coord made " << nb_iterations << " iterations.\n";
                        return true;
                    }                                  
                }
                size_t index = cnode.partial_conf.size();
                Agent agent = linearized[index]->agent_to_eliminate();
                auto successors = linearized[index]->getValueVector(cnode.partial_conf);
                if (successors.size() == 0)
                {
                    continue;
                }
                std::unordered_set<Node> node_support;
                for( auto p : cnode.partial_conf){
                    node_support.insert(p.second);
                }
                for (auto p : successors)
                {
                    Node suc_node = p.second;
                    // If a node was selected by another agent, we skip it
                    if (this->instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS && node_support.contains(suc_node))
                    {
                        continue;
                    }
                    // Otherwise, select it
                    size_t new_g = cnode.getG();
                    size_t suc_dist = p.first.first;
                    if (suc_node != source[agent]){
                        new_g++;
                    }
                    CoordNode new_node(cnode.partial_conf, suc_dist, new_g);
                    // std::cout <<"\tg=" << new_node.getG() <<" and cost=" << new_node.getCost() <<"\n";
                    new_node.partial_conf[agent] = suc_node;
                    open.push(new_node);

                    // std::cout << "Adding new node: ";
                    // for (auto p : new_node.partial_conf)
                    // {
                    //     std::cout << "Agent " << p.first << " @ " << p.second << ", ";
                    // }
                    // std::cout << "> with dist: " << new_node.getDistance() << " and g: " << new_node.getG() << "\n";

                }
            }
            return false;
        }


        Configuration get_next_best(const Configuration &source, const Configuration &goal, bool &success)
        {
            #ifdef COORD_DEBUG
            std::cout << ANSI_BLUE << "\nget_next_best\n" << ANSI_RESET;
            #endif
            initialize_qfuncs(source, goal);
            // std::cout << "Reducing...\n";
            for (int i = 0; i < this->instance_.nb_agents(); i++)
            {
                // for (auto qf : qfuncs_)
                // {
                //     std::cerr << "[";
                //     for (auto i : qf->arguments())
                //     {
                //         std::cerr << i << " ";
                //     }
                //     std::cerr << "] ";
                // }
                // std::cerr << "\n";
                reduce_qfuncs(i);
            }

            assert(qfuncs_.size() == 1);
            assert(qfuncs_[0]->arguments().size() == 0);
            assert((dynamic_cast<LocalQCompound<GraphMove, GraphComm> *>(qfuncs_[0].get()) != nullptr));

            // Obtain a list of compound Q-functions in topological order
            // (each qfunc depends only on the variables determined by the functions on its left)
            std::vector<std::shared_ptr<LocalQCompound<GraphMove, GraphComm>>> linearized;
            std::queue<std::shared_ptr<LocalQCompound<GraphMove, GraphComm>>> open;
            open.push(std::dynamic_pointer_cast<LocalQCompound<GraphMove, GraphComm>>(qfuncs_[0]));
            while (!open.empty())
            {
                std::shared_ptr<LocalQCompound<GraphMove, GraphComm>> qf = open.front();
                open.pop();
                linearized.push_back(qf);
                for (auto child_qf : qf->qfunctions())
                {
                    if (dynamic_cast<LocalQCompound<GraphMove, GraphComm> *>(child_qf.get()) != nullptr)
                    {
                        // std::shared_ptr<B> bp = std::dynamic_pointer_cast<B>(ap);
                        open.push(std::dynamic_pointer_cast<LocalQCompound<GraphMove, GraphComm>>(child_qf));
                    }
                }
            }

            // An agent is eliminated at each compound q-function, so there must be exactly nb_agents such objects
            assert(linearized.size() == this->instance().nb_agents());

            // Choose the best successor
            std::map<Agent, Node> partial_conf;
            std::set<Node> node_support;
            success = get_next_best_rec(source, linearized, partial_conf);
            Configuration successor(source.size());
            if (success)
            {
                for (auto p : partial_conf)
                {
                    successor[p.first] = p.second;
                }
            }
            return successor;
        }

    public:
        /**
         * @brief
         * @pre The following sets of agents are connected in the start configuration:
         * [0 1 .. window_size - 1], [window_size-1 window_size-2 ... 2(window_size-1)], [2(window_size-1) ... 3(window_size-1)], etc.
         */

        CoordSolver(const Instance<GraphMove, GraphComm> &instance,
                    const Objective &objective,
                    Heuristics<GraphMove, GraphComm> &heuristics,
                    unsigned int window_size)
            : Solver<GraphMove, GraphComm>(instance, objective),
              window_size_(window_size), heuristics_(heuristics)
        {
            config_stack_.push_back(instance.start());
            if (window_size < 2)
            {
                throw std::runtime_error("Windows size for CoordSolver must be at least 2");
            }
        }

        static bool isConfigurationWindowConnected(const Configuration &c, const Instance<GraphMove, GraphComm> &instance, int window_size)
        {
            Configuration localView;
            int local_index = 0;
            for (size_t i = 0; i < instance.nb_agents(); i++)
            {
                localView.push_back(c.at(i));
                if ( (i % (window_size-1)) == 0)
                {
                    // std::cout << "Checking block: ";
                    // for (auto n : localView){
                    //     std::cout << n << " ";
                    // }
                    // std::cout << "\n";
                    if (!instance.graph().communication().isConfigurationConnected(localView))
                    {
                        // std::cout << "Agent " << i << " is not connected to its block: ";
                        return false;
                    }
                    localView.clear();
                    localView.push_back(c.at(i));
                    local_index = 0;
                }
                local_index++;
            }
            return true;
        }

        /**
         * @brief Make greedy computation using get_next_best until goal is reached
         * @return the obtained execution
         */
        virtual const Execution compute() override
        {
            // int _iterations = 0;
            bool goal_reached = false;
            closed_.clear();
            while (!goal_reached)
            {
                //_iterations++;
                // Solution found?
                if (config_stack_.back() == this->instance_.goal())
                {
                    for (int i = 0; i < this->instance_.nb_agents(); i++)
                    {
                        std::shared_ptr<Path> path = std::make_shared<Path>();
                        for (auto c : config_stack_)
                        {
                            path->push_back(c[i]);
                        }
                        this->execution_.push_back(path);
                    }
                    // print
                    /*
                    for(auto c : config_stack_){
                        std::cerr << "[";
                        for(int i = 0; i< c.size();i++){
                            auto n = c[i];
                            auto pos = this->instance_.graph().movement().get_position(n);
                            std::cerr << "(node" << n << ", x=" << pos.second << ", y=" << pos.first << ") ";
                        }
                        std::cerr << "]\n";
                    }
                    */
                    goal_reached = true;
                    break;
                }
                bool success = true;
                Configuration next = get_next_best(config_stack_.back(), this->instance_.goal(), success);

                // std::cout << "<";
                // for(auto n : next){
                //     std::cout << n << ", ";
                // }
                // std::cout << ">\n";
                // if (_iterations % 20 == 0){
                //     Execution ex;
                //     std::vector<std::shared_ptr<Configuration>> paths;

                //     for (auto c : config_stack_){
                //         paths.push_back(std::make_shared<Configuration>(c));
                //     }
                //     ex.setPaths(paths);
                //     std::cout << ex;
                //     std::cout << "\n";
                // }
                /*
                std::cerr << "Iteration " << _iterations << ": " << next << std::endl;
                for(int i = 0; i< next.size();i++){
                    auto n = next[i];
                    auto pos = this->instance_.graph().movement().getPosition(n);
                    std::cerr << "(node" << n << ", x=" << pos.second << ", y=" << pos.first << ") ";
                }
                std::cerr << "]\n";
                */
                if (!success)
                {
                    if (config_stack_.size() == 1)
                    {
                        break;
                    }
                    else
                    {
                        // No successor found for config_stack_.back(). Backtracking.
                        config_stack_.pop_back();
                    }
                }
                else
                {
                    config_stack_.push_back(next);
                }
            }
            return this->execution_;
        }

        /**
         * @brief
         * @pre The following sets of agaents are connected in the source configuration:
         * [0 1 .. window_size - 1], [window_size-1 window_size-2 ... 2(window_size-1)], [2(window_size-1) ... 3(window_size-1)], etc.
         * Do not use configurations in excluded
         */
        std::vector<std::shared_ptr<Configuration>> computeBoundedPathTowards(const Configuration &source, const Configuration &goal,
                                                                              int steps, int max_iterations=-1)
        {
            bool success;
            this->max_iterations = max_iterations;
            this->nb_iterations = 0;
            if (this->bounded_queries_.count(std::make_pair(source,goal)) > 0){
                return this->bounded_queries_[std::make_pair(source,goal)];
            }
            //closed_.clear();
            closed_.insert(source);
            std::vector<std::shared_ptr<Configuration>> pathSegment{std::make_shared<Configuration>(source)};
            for (int i = 0; i < steps; i++)
            {
                Configuration next = get_next_best(*pathSegment.back().get(), goal, success);
                if (!success)
                    break;
                pathSegment.push_back(std::make_shared<Configuration>(next));
                if (next == goal)
                    break;
            }
            // Return empty path if source is the only element
            if (pathSegment.size() == 1)
            {
                pathSegment.clear();
            }
            this->bounded_queries_[std::make_pair(source,goal)] = pathSegment;
            return pathSegment;
        }

        unsigned int getWindowSize() const
        {
            return this->window_size_;
        }
        void test()
        {
            /*
            const Configuration & start = this->instance_.start();
            const Configuration & goal = this->instance_.goal();
            for (int i = 0; i < start.size(); i++){
                std::cerr << "(" << start[i] << " -- ";
                std::cerr << goal[i] << "), ";        }
            std::cerr <<"\n";

            this->initialize_qfuncs(this->instance_.start());
            std::map<Agent,Node> next_conf;
            next_conf[0] = start[0];
            next_conf[1] = start[1];
            next_conf[2] = start[2];
            // next_conf[0] = goal[0];
            // next_conf[1] = goal[1];
            // next_conf[2] = goal[2];
            std::cerr << "qfuncs size: " << qfuncs_.size() << "\n";
            std::cerr << "qfuncs[0]->getValue: " << qfuncs_[0]->getValue(next_conf) << "\n";
            std::cerr << "qfuncs[0]->getValue: " << qfuncs_[1]->getValue(next_conf) << "\n";
            std::cerr << "qfuncs[0]->getValue: " << qfuncs_[2]->getValue(next_conf) << "\n";
            std::vector<std::shared_ptr<LocalQ<GraphMove,GraphComm> > > my_qfuncs;
            my_qfuncs.push_back(qfuncs_[0]);
            my_qfuncs.push_back(qfuncs_[1]);
            std::shared_ptr<LocalQCompound<GraphMove,GraphComm> >qf_comp =
                std::make_shared<LocalQCompound<GraphMove,GraphComm> >(this->instance_, this->fw_,
                                                        qfuncs_[0]->getSourceConfiguration(),
                                                        my_qfuncs,
                                                        1,
                                                        collision_mode_);
            std::cerr << "qcomp has arguments: ";
            for(auto i : qf_comp->arguments()){
                std::cerr << i << " ";
            }
            std::cerr << "\n";
            qf_comp->print();
            std::cerr << "qcomp.getValue: " << qf_comp->getValue(next_conf) << "\n";

            my_qfuncs.clear();
            my_qfuncs.push_back(qf_comp);
            my_qfuncs.push_back(qfuncs_[2]);
            std::shared_ptr<LocalQCompound<GraphMove,GraphComm> >qf_comp2 =
                std::make_shared<LocalQCompound<GraphMove,GraphComm> >(this->instance_, this->fw_,
                                                        qfuncs_[0]->getSourceConfiguration(),
                                                        my_qfuncs,
                                                        2,
                                                        collision_mode_);
            qf_comp2->print();
            std::cerr << "qcomp.getValue: " << qf_comp2->getValue(next_conf) << "\n";

            my_qfuncs.clear();
            my_qfuncs.push_back(qf_comp2);
            std::shared_ptr<LocalQCompound<GraphMove,GraphComm> >qf_comp3 =
                std::make_shared<LocalQCompound<GraphMove,GraphComm> >(this->instance_, this->fw_,
                                                        qfuncs_[0]->getSourceConfiguration(),
                                                        my_qfuncs,
                                                        0,
                                                        collision_mode_);
            qf_comp3->print();
            std::cerr << "qcomp.getValue: " << qf_comp3->getValue(next_conf) << "\n";


            std::map<Agent,Node> partial_conf;
            std::pair<size_t,Node> best = qf_comp3->getValueVector(partial_conf)[0];
            std::cerr << "Agent " << qf_comp3->agent_to_eliminate() << " -> Node " << best.second <<
                " (with val=" << best.first << ")\n";
            partial_conf[qf_comp3->agent_to_eliminate()] = best.second;

            best = qf_comp2->getValueVector(partial_conf)[0];
            std::cerr << "Agent " << qf_comp2->agent_to_eliminate() << " -> Node " << best.second <<
                " (with val=" << best.first << ")\n";
            partial_conf[qf_comp2->agent_to_eliminate()] = best.second;

            best = qf_comp->getValueVector(partial_conf)[0];
            std::cerr << "Agent " << qf_comp->agent_to_eliminate() << " -> Node " << best.second <<
                " (with val=" << best.first << ")\n";
            */

            auto graph = this->instance_.graph().movement();
            const Configuration &start = this->instance_.start();
            const Configuration &goal = this->instance_.goal();
            for (int i = 0; i < start.size(); i++)
            {
                std::cerr << "(" << start[i] << " -- ";
                std::cerr << goal[i] << "), ";
            }
            std::cerr << "\n";
            std::cerr << "Neighborhood of node " << start[0] << "\n";
            auto neigh = graph.get_neighbors(start[0]);
            for (auto n : neigh)
            {
                std::cerr << n << " ";
            }
            std::cerr << "\n";
            /*
            const std::vector<Agent> args ={0,1,2};
            DijkstraSPCalculator<GraphMove,GraphComm> fw(this->instance_);
            LocalQ<GraphMove,GraphComm> localQ(start, args, fw, this->instance_);
            assert(localQ.isCollisionless(start));
            assert(localQ.isCollisionless(goal));
            assert(localQ.isConnected(start));
            assert(localQ.isConnected(goal));
            localQ.getValue(start);
            */
        }
    };
}