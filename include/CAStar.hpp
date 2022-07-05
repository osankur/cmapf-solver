/* Copyright (c) 2022 Ocan Sankur
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

#include <Execution.hpp>
#include <Instance.hpp>
#include <Objective.hpp>
#include <Solver.hpp>
#include <DFS.hpp>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <ShortestPathCalculator.hpp>
#include <list>
namespace decoupled{

template <class GraphMove, class GraphComm>
class CAStar : public Solver<GraphMove, GraphComm>
{
private:
    coupled::DFS<GraphMove, GraphComm> dfs_solver_;
    std::vector<Configuration> config_stack_;
    DijkstraSPCalculator<GraphMove,GraphComm> dijkstra_;
    Heuristics<GraphMove, GraphComm> & heuristics_;
    bool verbose_ = false;
    const int NUMBER_OF_TRIALS = 50;
public:
    CAStar(const Instance<GraphMove, GraphComm> &instance, const Objective &objective, Heuristics<GraphMove, GraphComm>& heuristics, bool verbose)
        : Solver<GraphMove, GraphComm>(instance, objective), dfs_solver_(instance, objective, heuristics, false), dijkstra_(instance), heuristics_(heuristics), verbose_(verbose) {}
    ~CAStar() {}

private:

        class CANode {
            public:
            CANode(Node node, Node parent, size_t dist, size_t g) : node(node), parent(parent), dist(dist), g(g){
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
            Node node;
            Node parent;
            size_t dist;
            size_t g;
        };
        struct CANodeCmp
        {
            bool operator()(const CANode & a, const CANode & b) const
            {
                return (a.getCost() > b.getCost() ||
                        a.getCost() == b.getCost() && a.getG() < b.getG() ||
                        a.getCost() == b.getCost() && a.getG() == b.getG() && &a < &b);
            }
        };
    /**
     * @brief Find a shortest path for Agent i from start to goal while staying inside neighborhoods[t] at time t.
     *        By construction, staying inside neighborhoods must ensure connectivity with agents 0..i, and collision-freedom.
     * @param i 
     * @param neighborhoods 
     * @return Path 
     */
    Path getConnectedPathTowardsGoal(Agent agt, std::vector<std::unordered_set<Node>> & neighborhoods){
        bool local_verbose = false;
        Node start = this->instance_.start()[agt];
        Node goal = this->instance_.goal()[agt];
        CANodeCmp cmp;
        std::priority_queue<CANode, std::vector<CANode>, CANodeCmp> open(cmp);
        std::unordered_map<Node,Node> parent;
        std::unordered_map<Node,size_t> distance;

        // size_t best_cost =  std::numeric_limits<int>::max();
        size_t best_h =  std::numeric_limits<int>::max();
        Node best_node = 0;

        size_t maxg = neighborhoods.size()-1;

        CANode init_node(start, start, this->heuristics_.getHeuristic(start, goal), 0);
        open.push(init_node);
        if (local_verbose)
            std::cout << "initially node " << start << " has h = " << init_node.getDistance() << " parent: " << init_node.parent << "(goal = " << goal << "\n";
        int nb_iterations = 0;
        while(!open.empty()){
            nb_iterations++;
            if (nb_iterations % 100 == 0){
                // std::cout << "#" << nb_iterations << "\n";
                std::cout.flush();
            }

            CANode cnode = open.top();
            open.pop();
            if (distance.count(cnode.node) > 0){
                if (distance[cnode.node] <= cnode.getDistance())
                    continue;
            }
            parent[cnode.node] = cnode.parent;
            distance[cnode.node] = cnode.getDistance();
            if (cnode.node == goal){
                best_node = goal;
                best_h = cnode.getDistance();
                // best_g = cnode.g;
                // std::cout << "Found goal!\n";
                break;
            //} else if ( cnode.g > best_g || cnode.g == best_g && cnode.dist < best_h) {
            } else if ( cnode.getDistance() < best_h) {
                best_node = cnode.node;
                best_h = cnode.getDistance();
                if (local_verbose)
                    std::cout << "Updating best_h:" << best_h << "\n";
                // best_g = cnode.g;
                // best_h = cnode.dist;
            }
            if (local_verbose)
                std::cout << "Popped: " << cnode.node << " h=" << cnode.dist << " g=" << cnode.g << " parent: " << cnode.parent << " \n";
            if(cnode.getG() + 1 >= neighborhoods.size()){                
                for(int i = neighborhoods.size(); i <= cnode.getG()+1; i++){
                    neighborhoods.push_back(neighborhoods.back());
                }
                //if (local_verbose)
                // std::cout << ANSI_RED << "Extending neighborhoods to " << neighborhoods.size() << "\n" << ANSI_RESET;
            }
            for (Node sucnode : this->instance_.graph().movement().get_neighbors(cnode.node)){
                assert(cnode.getG()+1 < neighborhoods.size());
                if (neighborhoods.at(cnode.getG()+1 ).contains(sucnode)){
                    CANode newnode(sucnode, cnode.node, this->heuristics_.getHeuristic(sucnode, goal), cnode.getG()+1);
                    open.push(newnode);
                    if (local_verbose)
                        std::cout << "\tPushing: " << newnode.node << " (with parent: " << newnode.parent << ")\n";
                }
            }
        }
        Path p;
        std::list<Node> path_list;
        Node current = best_node;
        int count = 0;
        path_list.push_front(current);
        while( current != start){
            current = parent[current];
            path_list.push_front(current);
        }
        // reverse it
        for (auto node : path_list){
            p.push_back(node);
        }
        return p;
    }

    /**
     * @brief Pick random order, fill in config_stack_ by a path of Agent 1, then Agent 2 connected to 1, then 3 connected to 1,2 etc.
     * 
     */
    void computePrefix()
    {

        const Configuration & start = this->instance_.start();
        const Configuration & goal = this->instance_.goal();
        std::vector<Agent> shuffled;
        for(Agent agt = 0; agt < this->instance_.nb_agents(); agt++){
            shuffled.push_back(agt);
        }
        std::random_shuffle(shuffled.begin(), shuffled.end());

        std::unordered_set<Node> cluster = this->instance_.graph().communication().get_neighbors(start.at(shuffled.at(0)));
        for(Agent i = 1; i < this->instance_.nb_agents(); i++){
            // Make sure that shuffle[agt] is connected to shuffle[0...agt-1], swap with someone if needed
            Agent j = i;
            while (!cluster.contains(start.at(shuffled.at(j))) && j < this->instance_.nb_agents()){
                j++;
            }
            assert(cluster.contains(start.at(shuffled.at(j))));
            Agent tmp = shuffled.at(i);
            shuffled[i] = shuffled.at(j);
            shuffled[j] = tmp;
            for(auto node : this->instance_.graph().communication().get_neighbors(start[shuffled[i]])){
                cluster.insert(node);
            }
        }

        // Safety check
        cluster = this->instance_.graph().communication().get_neighbors(start[shuffled[0]]);
        for(Agent i = 1; i < this->instance_.nb_agents(); i++){
            assert(cluster.contains(start[shuffled[i]]));
            for(auto node : this->instance_.graph().communication().get_neighbors(start[shuffled[i]])){
                cluster.insert(node);
            }
        }

        // Add first path
        std::vector<Path> paths;
        paths.push_back(this->dijkstra_.getShortestPath(start.at(shuffled.at(0)), goal.at(shuffled.at(0))));
        int path_size = paths.back().size();

        // std::cout << ANSI_GREEN << "Got path for Agent " << shuffled[0] << " (number " << 0 << ") of size : " << paths.back().size() << ". From "
        //     << this->instance_.start()[shuffled[0]] << " to " << this->instance_.goal()[shuffled[0]] << "\n<" << ANSI_RESET;
        // for(auto node : paths.back()){
        //     std::cout << node << ",";
        // }
        // std::cout <<">\n";
        
        // Initialize neighborhoods
        std::vector<std::unordered_set<Node>> neighborhoods;
        for(int t = 0; t < paths.back().size(); t++){
            neighborhoods.push_back(std::unordered_set<Node>());
        }
        for(int t = 0; t < paths.back().size(); t++){
            // Add the comm. neighborhood of the new path
            for(auto node : this->instance_.graph().communication().get_neighbors(paths.back().at(t))){
                neighborhoods.at(t).insert(node);
            }
            if (this->instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS ){
                neighborhoods[t].erase(paths.back().at(t));
            }
            // std::cout << "Neighborhood[" << t<< "] = ";
            // for(auto node : neighborhoods[t]){
            //     std::cout << node << " ";
            // }
            // std::cout << "\n";
        }
        // Add other paths one by one
        for(Agent i = 1; i < this->instance_.nb_agents(); i++){
            paths.push_back(this->getConnectedPathTowardsGoal(shuffled[i], neighborhoods));
            if(paths.back().size() > path_size){
                path_size = paths.back().size();
            }

            // std::cout << ANSI_GREEN << "Got path for Agent " << shuffled[i] << " (number " << i << ") of size : " << paths.back().size() << ANSI_RESET
            //     << " from " << this->instance_.start()[shuffled[i]] << " to " << this->instance_.goal()[shuffled[i]] << "\n<";
            // for(auto node : paths.back()){
            //     std::cout << node << ",";
            // }
            // std::cout <<">\n";

            for(int t = 0; t < neighborhoods.size(); t++){
                // Add the comm. neighborhood of the new path
                for(auto node : this->instance_.graph().communication().get_neighbors(paths.back().getAtTimeOrLast(t))){
                    neighborhoods[t].insert(node);
                }
                if (this->instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS ){
                    for (Agent j = 0; j < i; j++){
                        neighborhoods[t].erase(paths[j].getAtTimeOrLast(t));
                    }
                }
            }
        }

        // // Try to fix failed paths
        // for(Agent i = 1; i < this->instance_.nb_agents(); i++){
        //     if (paths[shuffled[i]] == this->instance_.goal()[shuffled[i]]){
        //         continue;
        //     }
        //     std::cout << ANSI_BLUE << "Attempting to fix " << shuffled[i] << "\n";
        //     // recompute neighborhoods
        //     for(int t = 0; t < neighborhoods.size(); t++){
        //         neighborhoods[t] = std::unordered_set<Node>();
        //     }
        //     for(int t = 0; t < paths.back().size(); t++){
        //         for (Agent agt = 0; agt < this->instance_.nb_agents(); agt++){
        //             if (shuffled[i] == agt) continue;
        //             // Add the comm. neighborhood of the new path
        //             for(auto node : this->instance_.graph().communication().get_neighbors(paths.at(agt).getAtTimeOrLast(t))){
        //                 neighborhoods.at(t).insert(node);
        //             }
        //         }
        //         if (this->instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS ){
        //             for (Agent agt = 0; agt < this->instance_.nb_agents(); agt++){
        //                 if (shuffled[i] == agt) continue;
        //                     neighborhoods[t].erase(paths.at(agt).getAtTimeOrLast(t));
        //             }
        //         }
        //     }
        //     paths[i] = this->getConnectedPathTowardsGoal(shuffled[i], neighborhoods);
        //     std::cout << ANSI_BLUE << "Updated path for Agent " << shuffled[i] << " (number " << i << ") of size : " << paths.back().size() << "\n<" << ANSI_RESET
        //         << this->instance_.start()[shuffled[i]] << " to " << this->instance_.goal()[shuffled[i]] << "\n<" << ANSI_RESET;

        // }
        
        // Convert to vector of Configurations
        for(int t = 0; t < path_size; t++){
            Configuration c(this->instance_.nb_agents());
            for(Agent i = 0; i < this->instance_.nb_agents(); i++){
                c[shuffled[i]] = paths[i].getAtTimeOrLast(t);
            }
            if (this->instance_.graph().communication().isConfigurationConnected(c)){
                config_stack_.push_back(c);
                // std::cout << ANSI_YELLOW << c;
                // std::cout << "\n";
            } else {
                // std::cout << ANSI_RED << "The following configuration at t=" << t << " becomes disconnected: " << c;
                // std::cout <<"\n" << ANSI_RESET;
                break;
            }
        }
    }
    const Execution compute() override
    {
        std::vector<Configuration> bestPrefix;

        for(int i = 0; i < this->NUMBER_OF_TRIALS; i++){
            config_stack_.clear();
            // std::cout << ANSI_PURPLE << "\n\ncomputePrefix:\n" << ANSI_RESET;
            computePrefix();
            // std::cout << ANSI_PURPLE << "Got: " << config_stack_.size() << ANSI_RESET;
            if (this->config_stack_.size() > bestPrefix.size()){
                bestPrefix = this->config_stack_;
                std::cout << ANSI_CYAN << "Got new prefix of length " << bestPrefix.size() << "\n" << ANSI_RESET;
                std::cout << "\t which ends in " << bestPrefix.back();
            }
            if (config_stack_.back() == this->instance_.goal()){
                break;
            }
            // return Execution();
        }
        std::vector<std::shared_ptr<Configuration> > suffix;
        if (config_stack_.back() != this->instance_.goal()){
            if(verbose_){
                std::cout << ANSI_CYAN << "Keeping prefix of length " << bestPrefix.size() << "\n" << ANSI_RESET;
                std::cout << ANSI_BLUE << "Now running DFS from " << bestPrefix.back();
                std::cout << "\t towards " << this->instance_.goal();
                std::cout << "\n" << ANSI_RESET;
            }
            suffix = this->dfs_solver_.computeBoundedPathTowards(bestPrefix.back(), this->instance_.goal(), 1000000, false);
            std::cout << ANSI_CYAN << "Got suffix of length " << suffix.size() << "\n" << ANSI_RESET;
        }
        for (int i = 0; i < this->instance_.nb_agents(); i++)
        {
            std::shared_ptr<Path> path = std::make_shared<Path>();
            for (auto c : bestPrefix)
            {
                path->push_back(c[i]);
            }
            if (suffix.size() > 0){
                for (auto it = suffix.begin()+1; it != suffix.end(); it++)
                {
                    path->push_back((*it)->at(i));
                }
            }
            assert(path->isValid(this->instance_.graph().movement()));
            this->execution_.push_back(path);
        }
        return this->execution_;

        // if ( suffix.size() == 0) return Execution();
        // // Success
        // if (*suffix.back() == this->instance_.goal()){
        //     auto it = suffix.begin()+1; // copy by ignoring the first element which is duplicated
        //     for(; it != suffix.end(); it++){
        //         config_stack_.push_back(**it);
        //     }
        //     for (int i = 0; i < this->instance_.nb_agents(); i++)
        //     {
        //         std::shared_ptr<Path> path = std::make_shared<Path>();
        //         for (auto c : config_stack_)
        //         {
        //             path->push_back(c[i]);
        //         }
        //         this->execution_.push_back(path);
        //     }
        // } else {
        //     return Execution();
        // }
    }

};
}