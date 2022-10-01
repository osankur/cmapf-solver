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

/**
 * @brief CA* algorithm. The compute method attempts to find the longest connected and collision-free execution towards the goal configuration,
 * by trying \a number_of_trials different random orderings of the agents. The longest computed prefix is stored. Then we try to finish with the
 * DFS solver.
 * 
 * @tparam GraphMove 
 * @tparam GraphComm 
 */

template <class GraphMove, class GraphComm>
class CAStar : public Solver<GraphMove, GraphComm>
{
private:
    coupled::DFS<GraphMove, GraphComm> dfs_solver_;
    std::vector<Configuration> config_stack_;
    DijkstraSPCalculator<GraphMove,GraphComm> dijkstra_;
    Heuristics<GraphMove, GraphComm> & heuristics_;
    bool verbose_ = false;
    int number_of_trials_ = 100;
    int number_of_subtrials_ = 100;
public:
    CAStar(const Instance<GraphMove, GraphComm> &instance, const Objective &objective, Heuristics<GraphMove, GraphComm>& heuristics, int number_of_trials, int number_of_subtrials, bool verbose)
        : Solver<GraphMove, GraphComm>(instance, objective), dfs_solver_(instance, objective, heuristics, false), dijkstra_(instance), heuristics_(heuristics), number_of_trials_(number_of_trials), number_of_subtrials_(number_of_subtrials),verbose_(verbose) {}
    ~CAStar() {}

private:

        /**
         * @brief Serch node for the CA* algorithm. Each node is given with its successor, whether from the successor
         * there is an actual path to goal (\a has_path), the length of the feasible suffix from the current node (\a suffix_length),
         * the h value (feasible suffix + heuristic value), and whether the current node has no successor in the next frame (\a dead_end)
         * 
         */
        class CANode {
            public:
            CANode(){}
            CANode(Node node, Node successor, bool has_path, size_t suffix_length, size_t h, bool dead_end = false) : node(node), successor(successor), has_path(has_path), suffix_length(suffix_length), h(h), dead_end(dead_end){
            }
            Node node;
            Node successor;
            bool has_path;
            size_t suffix_length;
            size_t h;
            bool dead_end;
            bool operator<(const CANode & b) const
            {
                return this->has_path > b.has_path
                        || this->has_path && b.has_path && (this->suffix_length < b.suffix_length)
                        || !this->has_path && !b.has_path && (b.suffix_length < this->suffix_length)
                        || this->has_path == b.has_path && (this->suffix_length == b.suffix_length) && this->h < b.h;
            }
            void print(std::ostream & os){
                os << "* canode " << node << " -> " 
                                << successor << ", has_path: " << has_path << ", suffix_length: " << suffix_length
                                << ", h: " << h << "\n";        
            }
        };

    /**
     * @brief Find the longest such that at each step t, the node is in \a neighborhoods[t], while either actually reaching goal,
     * or minimizing the h-value to goal at the last vertex.
     *
     * Each set neighborhoods[t] is called a frame. We first compute the set of nodes in the last frame which has a path to goal
     * which stays in this frame. These nodes have an actual path to goal; and the execution can be extended for agt to reach goal
     * by making all other agents idle.
     * Then we compute the best path from each node of each frame backwards. At each frame t, and node n, we assign to n the successor
     * from frame t+1 which has, if possible, an actual path to goal and in this case the one admitting the shortest one,
     * and otherwise a successor node with the longest path with the least h value. Some nodes of frame t might have no successors in
     * frame t+1. These are marked as deadends, and assigned a heuristics h value.
     * At the end of the computation, the start node at frame 0 has been assigned the best path. We follow this path, and possibly 
     * complete it with a suffix path within the last frame.
     * 
     * @pre Agent \a agt staying inside \a neighborhoods must ensure connectivity with agents 0 .. \a agt -1, and collision-freedom.
     * @param i 
     * @param neighborhoods 
     * @return Path 
     */
    Path getConnectedPathTowardsGoal(Configuration startConf, Agent agt, std::vector<std::unordered_set<Node>> & neighborhoods){
        bool local_verbose = false;
        Node start = startConf[agt];
        Node goal = this->instance_.goal()[agt];
        Path p;
        
        // std::cout << "\n* getConnectedPathTowardsGoal(Agt: " << agt << " Start: " << start << ", goal: " << goal << ") with neighborhoods size: " <<
        //     neighborhoods.size() << "\n";
        // std::cout << "neighborhoods[0] = ";
        // for (auto node : neighborhoods[0]){
        //     std::cout << node << " ";
        // }
        // std::cout << "\n";  

        // if (neighborhoods.size()>1){
        //     std::cout << "neighborhoods[1] = ";
        //     for (auto node : neighborhoods[1]){
        //         std::cout << node << " ";
        //     }
        //     std::cout << "\n";  
        // }

        // std::cout << "neighborhoods[" << neighborhoods.size()-1 << "] = ";
        // for (auto node : neighborhoods[neighborhoods.size()-1]){
        //     std::cout << node << " ";
        // }
        // std::cout << "\n";  
        assert(neighborhoods[0].contains(start));

        if (local_verbose){
            std::cout << "Agt: " << agt << " Start: " << start << ", goal: " << goal << "\n";
        }
        // We start by computing the suffix. Let suffix_h be the distance from each vertex of the last frame to goal,
        // while staying inside the said frame. Let suffix_next be the next node to go to follow this suffix.
        // The path that follows suffix_next corresponds to previous agents idling at their final positions while agt continuing towards their goal.

        std::unordered_map<Node,Node> suffix_next; // for a vertex of the last frame, the vertex to go to for agt
        std::unordered_map<Node,size_t> suffix_h; // for a vertex of the last frame, the distance to go for agt from given node to goal
        suffix_next[goal] = goal;
        suffix_h[goal] = 0;
        std::stack<Node> wait;
        std::unordered_set<Node> has_path_to_goal;
        // We do this step only if goal is in the last frame
        if (neighborhoods[neighborhoods.size()-1].contains(goal))
            wait.push(goal);

        while(!wait.empty()){
            Node current = wait.top();
            wait.pop();
            if (has_path_to_goal.contains(current))
                continue;
            has_path_to_goal.insert(current);
            for (Node sucnode : this->instance_.graph().movement().get_neighbors(current)){
                // for all movement neighbor node sucnode that are in the last frame
                if (neighborhoods[neighborhoods.size()-1].contains(sucnode) && !has_path_to_goal.contains(sucnode)){
                    suffix_next[sucnode] = current;
                    suffix_h[sucnode] = suffix_h[current] + 1;
                    if (local_verbose){
                        std::cout << "suffix_next[" << sucnode << "] = " << current << "\n";
                        std::cout << "suffix_h[" << sucnode << "] = " << suffix_h[sucnode] <<"\n";
                    }
                    wait.push(sucnode);
                }
            }
        }

        std::vector<std::unordered_map<Node,CANode> > prefix_h;
        for(int i=0; i < neighborhoods.size(); i++){
            prefix_h.push_back(std::unordered_map<Node,CANode>());
        }
        for(Node node : has_path_to_goal){
            prefix_h[neighborhoods.size()-1][node] = CANode(node, node, true, suffix_h[node], suffix_h[node]);
        }
        for (Node node : neighborhoods[neighborhoods.size()-1]){
            if (!has_path_to_goal.contains(node)){
                prefix_h[neighborhoods.size()-1][node] = CANode(node, node, false, 0, this->heuristics_.getHeuristic(node,goal));
            }
        }
        if (local_verbose){
            std::cout << "Last frame (index " << neighborhoods.size()-1 << ") is\n";
            for (Node node : neighborhoods[neighborhoods.size()-1]){
                CANode canode = prefix_h[neighborhoods.size()-1][node];
                canode.print(std::cout);
            }
        }
        for (int i = neighborhoods.size()-1; i> 0; i--){
            for (Node node : neighborhoods[i-1]){
                for (Node sucnode : this->instance_.graph().movement().get_neighbors(node)){
                    if (!neighborhoods[i].contains(sucnode))
                        continue;
                    assert(prefix_h[i].count(sucnode) != 0);
                    CANode sucCANode = prefix_h[i][sucnode];
                    // std::cout << "Successor is: "; sucCANode.print(std::cout);

                    CANode candidate(node, sucnode, sucCANode.has_path, sucCANode.suffix_length+1, sucCANode.h+1);
                    if (prefix_h[i-1].count(node) == 0 
                        || candidate < prefix_h[i-1][node])
                    {
                        prefix_h[i-1][node] = candidate;
                        if (local_verbose){
                            //std::cout << "prefix_next(" << i-1 << ")[node " << node << "] = node " << sucnode << "\n";
                            std::cout << "prefix_h[" << i-1 << "] "; candidate.print(std::cout);
                        }
                    }
                }
                // if node has no successors in neighborhoods[i], then it is a dead end
                if (prefix_h[i-1].count(node) == 0){
                    CANode candidate(node, node, false, 0, this->heuristics_.getHeuristic(node,goal), true);
                    prefix_h[i-1][node] = candidate;
                }
            }
        }

        assert(prefix_h[0].count(start) > 0);
        CANode canode = prefix_h[0][start];
        if (local_verbose)
            std::cout << "Prefix (" << start << " to " << goal << "): " << canode.node << " ";
        p.push_back(canode.node);
        for(int i = 1; !canode.dead_end && i < neighborhoods.size();i++){
            assert(prefix_h[i].count(canode.successor)>0);
            canode = prefix_h[i][canode.successor];
            if (local_verbose){
                std::cout << canode.node << "@" << i << " ";
                std::cout.flush();
            }
            p.push_back(canode.node);
        }

        // check
        for(int i = 0; i < p.size(); i++){
            assert(neighborhoods.at(i).contains(p[i]));
        }

        // if (!canode.dead_end){
        //     p.push_back(canode.successor);
        //     std::cout << canode.successor << "$" << neighborhoods.size() << " ";
        //     std::cout.flush();
        // }

        // check 
        Node node = p[0];
        for(int i = 1; i < p.size(); i++){
            if (!this->instance_.graph().movement().get_neighbors(node).contains(p[i])){
                std::cout << "Prefix Step " << i << " is not a neighbor of previous step\n";
                exit(-1);
            }
            node = p[i];
        }

        if (canode.has_path){
            if ( local_verbose){
                std::cout << " :: "; std::cout.flush();
            }
            Node node = canode.successor;
            while(node != goal){
                node = suffix_next[node];
                p.push_back(node);
                assert(neighborhoods[neighborhoods.size()-1].contains(node));
                if ( local_verbose)
                    std::cout << node << " ";
            }
        }
        if ( local_verbose){
            std::cout << "\n"; std::cout.flush();
        }
        node = p[0];
        for(int i = 1; i < p.size(); i++){
            if (!this->instance_.graph().movement().get_neighbors(node).contains(p[i])){
                std::cout << "Whole Step " << i << " is not a neighbor of previous step\n";
                std::cout << "node " << p[i] << " not neighbor of node" << node << "\n";
                exit(-1);
            }
            node = p[i];
        }
        return p;
    }

    /**
     * @brief This is a best-effort CA* procedure that attempts to find the longest path towards goal configuration:
     * Pick random order, fill in config_stack_ by a path of Agent 1, then Agent 2 connected to 1, then 3 connected to 1,2 etc.
     * 
     */
    std::vector<Configuration> computePrefix(const Configuration & start)
    {
        // std::cout << "* computePrefix(" << start;
        // std::cout << ")\n";

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

        // Safety check for debugging
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
        
        // Initialize neighborhoods: neighborhoods[t] is the set of unoccupied vertices that communicate with some vertex occupied by
        // some agent at time t. Any subsequent agent must be and can be somewhere in neighboorhoods[t].
        std::vector<std::unordered_set<Node>> neighborhoods(paths.back().size());
        // for(int t = 0; t < paths.back().size(); t++){
        //     neighborhoods.push_back(std::unordered_set<Node>());
        // }
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
            paths.push_back(this->getConnectedPathTowardsGoal(start, shuffled[i], neighborhoods));

            if(paths.back().size() > path_size){
                // If the added path is longer than the previous ones, extend the neighborhoods by assuming all others idle at their
                // last positions. We do not actually extend the paths, they are implicitly extended.
                path_size = paths.back().size();
                for (int t = neighborhoods.size(); t < path_size; t++){
                    neighborhoods.push_back(neighborhoods.back());
                }
            } else {
                // If the added path is shorter, than shorten all previous paths and neigborhoods
                path_size = paths.back().size();
                neighborhoods.resize(path_size);
                for(int i = 0; i <paths.size();i++){
                    paths[i].resize(path_size);
                }
            }
            // std::cout << "path size is " << path_size << "\n";

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
                    for (Agent j = 0; j <= i; j++){
                        neighborhoods[t].erase(paths.at(j).getAtTimeOrLast(t));
                    }
                }
            }
        }

        // Convert to vector of Configurations
        std::vector<Configuration> configSeq;
        for(int t = 0; t < path_size; t++){
            // std::cout << "paths[" << t << "].size() == " << paths[t].size() << "\n";
            Configuration c(this->instance_.nb_agents());
            for(Agent i = 0; i < this->instance_.nb_agents(); i++){
                c[shuffled[i]] = paths[i].getAtTimeOrLast(t);
            }
            if (this->instance_.graph().communication().isConfigurationConnected(c)){
                configSeq.push_back(c);
            } else {
                break;
            }
            if (this->instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS && c.hasCollisions()){
                std::cout << " [error] Collision at step " << t << ": " << c;
                std::cout <<"\n";
                assert(!c.hasCollisions());
            }
        }
        return configSeq;
    }
    const Execution compute() override
    {
        std::vector<Configuration> bestPrefix;
        bool goal_reached = false;
        for(int i = 0; i < this->number_of_trials_ && !goal_reached; i++){
            config_stack_.clear();
            // std::cout << ANSI_PURPLE << "\n\ncomputePrefix:\n" << ANSI_RESET;
            // for (int j = 0; j < 1; j++){
            //     const Configuration start = this->instance_.start();
            //     std::vector<Path> paths = computePrefix(start);
            // }
            Configuration start = this->instance_.start();
            std::vector<Configuration> prefix = computePrefix(start);
            // std::cout << "(got prefix of size " << prefix.size() << ")\n";
            // std::cout << " ending in " << prefix.back() << "\n";
            // Try to extend it further
            if (prefix.back() != this->instance_.goal()){
                for (int j = 0; j < this->number_of_subtrials_; j++){
                    std::vector<Configuration> segment = computePrefix(prefix.back());
                    if (segment.size()>1){
                        // std::cout << "(extending prefix by segment of size " << segment.size() << ")\n";
                        for(auto c = segment.begin()+1; c != segment.end(); c++){
                            prefix.push_back(*c);
                        }
                    }
                }
            }
            if (prefix.size() > bestPrefix.size()){
                bestPrefix = prefix;
                std::cout << ANSI_CYAN << "Got new prefix of length " << prefix.size() << "\n" << ANSI_RESET;
                std::cout << "\t" << bestPrefix.back();
                std::cout << "\n";
                if (bestPrefix.back() == this->instance_.goal()){
                    goal_reached = true;
                    break;
                }
            }
        }
        this->config_stack_ = bestPrefix;

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
        } else {
            std::cout << ANSI_GREEN << "CA* solved the instance alone.\n" << ANSI_RESET;
        }

        if (suffix.size() > 0){
            for (auto it = suffix.begin()+1; it != suffix.end(); it++){
                config_stack_.push_back(**it);
            }
        }

        // Check
        assert(this->config_stack_.front() == this->instance_.start());
        assert(this->config_stack_.back() == this->instance_.goal());
        for(auto c : this->config_stack_){
            this->instance_.graph().communication().isConfigurationConnected(c);
        }

        // Create the execution
        for (int i = 0; i < this->instance_.nb_agents(); i++)
        {
            std::shared_ptr<Path> path = std::make_shared<Path>();
            for (auto c : config_stack_)
            {
                path->push_back(c[i]);
            }
            // if (suffix.size() > 0){
            //     for (auto it = suffix.begin()+1; it != suffix.end(); it++)
            //     {
            //         path->push_back((*it)->at(i));
            //     }
            // }
            assert(path->isValid(this->instance_.graph().movement()));
            this->execution_.push_back(path);
        }


        return this->execution_;
    }

};
}