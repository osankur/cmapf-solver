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
public:
    CAStar(const Instance<GraphMove, GraphComm> &instance, const Objective &objective, Heuristics<GraphMove, GraphComm>& heuristics, int number_of_trials, bool verbose)
        : Solver<GraphMove, GraphComm>(instance, objective), dfs_solver_(instance, objective, heuristics, false), dijkstra_(instance), heuristics_(heuristics), number_of_trials_(number_of_trials), verbose_(verbose) {}
    ~CAStar() {}

private:

        // class CANode {
        //     public:
        //     CANode(Node node, Node parent, size_t dist, size_t g) : node(node), parent(parent), dist(dist), g(g){
        //     }
        //     size_t getCost() const{
        //         return dist+g;
        //     }
        //     size_t getDistance() const{
        //         return dist;
        //     }
        //     size_t getG() const{
        //         return g;
        //     }
        //     Node node;
        //     Node parent;
        //     size_t dist;
        //     size_t g;
        // };
        // struct CANodeCmp
        // {
        //     bool operator()(const CANode & a, const CANode & b) const
        //     {
        //         return (a.getCost() > b.getCost() ||
        //                 a.getCost() == b.getCost() && a.getG() < b.getG() ||
        //                 a.getCost() == b.getCost() && a.getG() == b.getG() && &a < &b);
        //     }
        // };

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
     * @brief Find path of length at least that of \a neighborhoods, which minimizes the H-value to goal at the last vertex,
     * while staying inside neighborhoods[t] at each step t.
     * @pre Agent \a agt staying inside \a neighborhoods must ensure connectivity with agents 0 .. \a agt -1, and collision-freedom.
     * @param i 
     * @param neighborhoods 
     * @return Path 
     */
    Path getConnectedPathTowardsGoal(Agent agt, std::vector<std::unordered_set<Node>> & neighborhoods){
        bool local_verbose = false;
        Node start = this->instance_.start()[agt];
        Node goal = this->instance_.goal()[agt];
        Path p;
        
        if (local_verbose){
            std::cout << "Agt: " << agt << "Start: " << start << ", goal: " << goal << "\n";
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
        for(int i = 1; !canode.dead_end && i < neighborhoods.size()-1;i++){
            assert(prefix_h[i].count(canode.successor)>0);
            canode = prefix_h[i][canode.successor];
            if (local_verbose){
                std::cout << canode.node << "@" << i << " ";
                std::cout.flush();
            }
            p.push_back(canode.node);
        }
        if (local_verbose){
            std::cout << canode.successor << "@" << neighborhoods.size() << " ";
            std::cout.flush();
        }
        p.push_back(canode.successor);
        Node node = p[0];
        for(int i = 1; i < p.size(); i++){
            if (!this->instance_.graph().movement().get_neighbors(node).contains(p[i])){
                std::cout << "Prefix Step " << i << " is not a neighbor of previous step\n";
                exit(-1);
            }
            node = p[i];
        }
        if (canode.has_path){
            if (local_verbose){
                std::cout << " :: \n"; std::cout.flush();
            }
            Node node = canode.successor;
            while(node != goal){
                node = suffix_next[node];
                p.push_back(node);
                if (local_verbose)
                    std::cout << node << " ";
            }
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
        // if (prefix_next[0].contains(start)){
        //     Node current = start;
        //     if (local_verbose){
        //         std::cout << "Found path in the first phase: " << start << " to " << goal << "\n";
        //         std::cout << "Length: " << neighborhoods.size() << "\n";
        //     }
        //     for(int i = 0; i < neighborhoods.size()-1; i++){
        //         p.push_back(current);
        //         if (local_verbose){
        //             std::cout << i << ". " << current << " (prefix_h[" << i <<
        //             "][" << current << "] = " << prefix_h[i][current] << ")\n";
        //         }
        //         current = prefix_next[i][current];
        //     }
        //     int i = neighborhoods.size()-1;
        //     if (local_verbose){
        //         std::cout << i << ". " << current << " (prefix_h[" << i <<
        //         "][" << current << "] = " << prefix_h[i][current] << ")\n";
        //     }
        //     p.push_back(current);
        //     if (has_path_to_goal.contains(current)){
        //         if (local_verbose){
        //             std::cout << "\n\tAnd suffix: ";
        //         }
        //         while (current != goal){
        //             current = suffix_next[current];
        //             p.push_back(current);
        //             if (local_verbose){
        //                 std::cout << current << " ";
        //             }
        //         }
        //         if (local_verbose)
        //             std::cout << "\n";
        //     }
        //     return p;
        // } 
        

        // prefix_next.clear();
        // prefix_h.clear();
        // for(int i=0; i < neighborhoods.size(); i++){
        //     prefix_next.push_back(std::unordered_map<Node,Node>());
        //     prefix_h.push_back(std::unordered_map<Node,size_t>());
        // }
        // // Initialize the last frame with heuristic h values
        // for(Node node : neighborhoods[neighborhoods.size()-1]){
        //     prefix_h[neighborhoods.size()-1][node] = this->heuristics_.getHeuristic(node, goal);
        // }
        // for(int i=neighborhoods.size()-2; i>= 0; i--){
        //     // Set the values of prefix_next and prefix_h by quantifying over successors in neighborhoods[i+1] for each vertex
        //     for(Node node : neighborhoods[i]){
        //         for (Node sucnode : this->instance_.graph().movement().get_neighbors(node)){
        //             if ( neighborhoods[i+1].contains(sucnode)
        //                 && (prefix_next[i].count(node) == 0 || prefix_h[i][node] > prefix[i+1][sucnode]+1))
        //             {
        //                     prefix_next[i][node] = sucnode;
        //                     prefix_h[i][node] = prefix_h[i+1][sucnode] + 1;
        //             }
        //         }
        //     }
        //     // For all nodes in neighborhoods[i] not having any successor in neighborhoods[i+1], set the prefix_h to their heuristic value
        //     for(Node node : neighborhoods[i]){
        //         if (prefix_h[i].count(node) == 0){
        //             prefix_h[i][node] = this->heuristics_.getHeuristic(node, goal);
        //         }
        //     }
        // }

        // TODO Here, we won't be able to reach goal but we do the exact same thing to minimize h-value at the last frame

        // else {
        //     p.push_back(start);
        // }

        std::cout.flush();
        return p;
    }

    // Path getConnectedPathTowardsGoal_dijkstra(Agent agt, std::vector<std::unordered_set<Node>> & neighborhoods){
    //     bool local_verbose = false;
    //     Node start = this->instance_.start()[agt];
    //     Node goal = this->instance_.goal()[agt];
    //     CANodeCmp cmp;
    //     std::priority_queue<CANode, std::vector<CANode>, CANodeCmp> open(cmp);
    //     std::unordered_map<Node,Node> parent;   // parent of a given node in the shortest path
    //     std::unordered_map<Node,size_t> distance; // the h-value from current node

    //     size_t best_h =  std::numeric_limits<int>::max();
    //     Node best_node = 0;

    //     size_t maxg = neighborhoods.size()-1;

    //     CANode init_node(start, start, this->heuristics_.getHeuristic(start, goal), 0);
    //     open.push(init_node);
    //     if (local_verbose)
    //         std::cout << "initially node " << start << " has h = " << init_node.getDistance() << " parent: " << init_node.parent << "(goal = " << goal << ")\n";
    //     int nb_iterations = 0;
    //     while(!open.empty()){
    //         nb_iterations++;
    //         // if (nb_iterations % 100 == 0){
    //         //     std::cout << "#" << nb_iterations << "\n";
    //         //     std::cout.flush();
    //         // }

    //         CANode cnode = open.top();
    //         open.pop();
    //         if (distance.count(cnode.node) > 0){
    //             if (distance[cnode.node] <= cnode.getDistance())
    //                 continue;
    //         }
    //         parent[cnode.node] = cnode.parent;
    //         distance[cnode.node] = cnode.getDistance();
    //         if (cnode.node == goal){
    //             best_node = goal;
    //             best_h = cnode.getDistance();
    //             assert(best_h == 0);
    //             break;
    //         //} else if ( cnode.g > best_g || cnode.g == best_g && cnode.dist < best_h) {
    //         } else if ( cnode.getDistance() < best_h) {
    //             best_node = cnode.node;
    //             best_h = cnode.getDistance();
    //             if (local_verbose)
    //                 std::cout << "Updating best_h:" << best_h << "\n";
    //             // best_g = cnode.g;
    //             // best_h = cnode.dist;
    //         }
    //         if (local_verbose)
    //             std::cout << "Popped: " << cnode.node << " h=" << cnode.dist << " g=" << cnode.g << " parent: " << cnode.parent << " \n";
    //         if(cnode.getG() + 1 >= neighborhoods.size()){
    //             for(int i = neighborhoods.size(); i <= cnode.getG()+1; i++){
    //                 neighborhoods.push_back(neighborhoods.back());
    //             }
    //             //if (local_verbose)
    //             // std::cout << ANSI_RED << "Extending neighborhoods to " << neighborhoods.size() << "\n" << ANSI_RESET;
    //         }
    //         for (Node sucnode : this->instance_.graph().movement().get_neighbors(cnode.node)){
    //             assert(cnode.getG()+1 < neighborhoods.size());
    //             if (neighborhoods.at(cnode.getG()+1 ).contains(sucnode)){
    //                 CANode newnode(sucnode, cnode.node, this->heuristics_.getHeuristic(sucnode, goal), cnode.getG()+1);
    //                 open.push(newnode);
    //                 if (local_verbose)
    //                     std::cout << "\tPushing: " << newnode.node << " (with parent: " << newnode.parent << ")\n";
    //             }
    //         }
    //     }
    //     Path p;
    //     std::list<Node> path_list;
    //     Node current = best_node;
    //     int count = 0;
    //     path_list.push_front(current);
    //     while( current != start){
    //         current = parent[current];
    //         path_list.push_front(current);
    //     }
    //     // reverse it
    //     for (auto node : path_list){
    //         p.push_back(node);
    //     }
    //     return p;
    // }

    /**
     * @brief This is a best-effort CA* procedure that attempts to find the longest path towards goal configuration:
     * Pick random order, fill in config_stack_ by a path of Agent 1, then Agent 2 connected to 1, then 3 connected to 1,2 etc.
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

        for(int i = 0; i < this->number_of_trials_; i++){
            config_stack_.clear();
            // std::cout << ANSI_PURPLE << "\n\ncomputePrefix:\n" << ANSI_RESET;
            computePrefix();
            // std::cout << ANSI_PURPLE << "Got: " << config_stack_.size() << ANSI_RESET;
            if (this->config_stack_.size() > bestPrefix.size()){
                bestPrefix = this->config_stack_;
                std::cout << ANSI_CYAN << "Got new prefix of length " << bestPrefix.size() << "\n" << ANSI_RESET;
                std::cout << "\t" << bestPrefix.back();
                std::cout << "\n";
            }
            if (config_stack_.back() == this->instance_.goal()){
                break;
            }
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

        // Create the execution
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
    }

};
}