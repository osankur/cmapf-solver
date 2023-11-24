/**
 * @file CMARRT.hpp
 * @author Isseïnie Calvic, Ocan Sankur
 * @brief 
 * @version 0.1
 * @date 2022-06-12
 * 
 * @copyright Copyright (c) 2022 Isseïnie Calvic, Ocan Sankur
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

#include <DFS.hpp>
#include <ShortestPathCalculator.hpp>
#include <Instance.hpp>
#include <Objective.hpp>
#include <Solver.hpp>
#include <Heuristics.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <stack>
#include <unordered_set>
#include <utility>
#include <vector>
#include <ctime>

namespace cmarrt
{
  /**
   * @class Solver using the connected Multi-agent Rapidly-exploring
   * random tree Algorithm (CMARRT). Inspired by RRT, the algorithm
   * builds an exploration tree whose nodes are connected configurations.
   * At each step, a random target connected configuration is chosen, and the
   * tree is expanded by adding an edge from the closest node in the tree 
   * to a configuration towards the target, by solving a bounded subinstance of the problem.
   * 
   */
  template <class GraphMove, class GraphComm>
  class CMARRT : public Solver<GraphMove, GraphComm>
  {
  private:
    // Number of iterations done by the algorithm
    int _iterations = 0;
    // Percentage of time the random target configuration is the goal configuration
    int _prob2goal;
    // Number of steps of the executions linking the nodes of the tree
    int _step_size;
    // Debug verbose mode
    bool _verbose = true;
    // Whether RRT* is activated
    bool _cmarrtstar = false;
    // Whether start and goal configurations are window-connected
    bool _window_connected = false;

    int max_subsolver_iterations_ = 50000;

    Heuristics<GraphMove, GraphComm> &heuristics_;

    SubsolverEnum subsolver;

    std::vector<std::shared_ptr<Configuration>> vertices_;
    std::vector<std::shared_ptr<Configuration>> parents_;
    std::map<Configuration, int> index_of_vertex_;
    std::map<Configuration, int> index_of_parent_;
    // given configuration c in the tree, path_from_parent_[c] is the path from the parent to itself
    std::map<Configuration, std::vector<std::shared_ptr<Configuration>>> path_from_parent_;

    // Min of the sum of SP distances of the vertices in the tree
    int _min_distance;

    // Set to true when segments computed towards goal end in configurations that are already in the tree
    bool towards_goal_becoming_redundant_ = false;
    // When towards_goal_becoming_redundant, we start picking a random configuration in the tree rather than the nearest.
    // This is the count of how many times we have been doing this.
    int towards_goal_randomize_count_ = 0;
    // After towards_goal_randomize_period many times we have randomize the nearby configuration for goal, we set towards_goal_becoming_redundant_ to false.
    const int towards_goal_randomize_period = 10;

    coupled::DFS<GraphMove, GraphComm> dfs_solver_;
    // decoupled::BoundedDecoupledSolver<GraphMove, GraphComm> decoupled_solver_;
    coordinated::CoordSolver<GraphMove, GraphComm> coord_solver_;

    int indexOfConfiguration(const Configuration &c)
    {
      if (index_of_vertex_.find(c) == index_of_vertex_.end())
      {
        return -1;
      }
      else
      {
        return index_of_vertex_[c];
      }
    };

    int parentIndexOfConfiguration(const Configuration &c)
    {
      if (index_of_parent_.find(c) == index_of_parent_.end())
      {
        return -1;
      }
      else
      {
        return index_of_parent_[c];
      }
    };

    /**
     * @brief returns a random target configuration: with probability _prob2goal%,
     * the goal configuration is picked; with remaining probability, a random connected
     * configuration is returned.
     *
     * @param goal
     * @return target configuration
     */
    std::shared_ptr<Configuration> getRandomConfiguration(
        const Configuration &goal)
    {
        // auto configuration = std::make_shared<Configuration>();
        // for (int agt = 0; agt < this->instance().nb_agents(); agt++)
        // {
        //   configuration->push_back(first);
        // }
        // return configuration;
        // Pick a node of Gc for the 1st agent
        size_t nb_nodes = this->instance().graph().communication().node_count();
        Node first = (uint64_t)rand() % (nb_nodes - 1);
        std::vector<Node> config = {first};
        std::unordered_set<Node> neighbors;
        // Pick in the Gc neighbors the position for the 2nd, etc
        for (size_t agt = 1; agt < this->instance().nb_agents(); agt++)
        {
          auto new_neighbors = this->instance().graph().communication().get_neighbors(
              config.at(agt - 1));
          neighbors.insert(new_neighbors.begin(), new_neighbors.end());

          auto neighbors_vector =
              std::vector<Node>(neighbors.begin(), neighbors.end());
          auto size = neighbors_vector.size();
          // std::cout << "size for agt " << agt << ":" << size;
          auto nextagt = rand() % size;
          config.push_back(neighbors_vector.at(nextagt));
          assert(neighbors_vector.at(nextagt) <
                 this->instance().graph().movement().node_count());
        }
        std::random_shuffle(config.begin(), config.end());
        auto configuration = std::make_shared<Configuration>();
        for (int agt = 0; agt < this->instance().nb_agents(); agt++)
        {
          configuration->push_back(config.at(agt));
        }
        return configuration;
    };

    /**
     * @brief Get the configuration of the tree that is the closest to rand,
     * using distances between the barycenters of configurations.
     *
     * Linear in the size of the tree.
     *
     * @todo Complexity can be improved with space partitioning
     *
     * @param rand target configuration
     * @return shared pointer to the nearest configuration in the tree
     */
    std::shared_ptr<Configuration> getNearestConfigurationByBarycenter(
        const Configuration &rand)
    {
      int mindist = -1;
      std::shared_ptr<Configuration> c_nearest = nullptr;
      std::pair<float, float> rand_pos = this->instance_.graph().movement().getBarycenter(rand);
      for (auto v : this->getVertices())
      {
        std::pair<float, float> v_pos = this->instance_.graph().movement().getBarycenter(*v);
        int dist = (abs(v_pos.first - rand_pos.first) +
                    abs(v_pos.second - rand_pos.second));
        if (dist < mindist || mindist == -1)
        {
          mindist = dist;
          c_nearest = v;
        }
      }
      assert(c_nearest);
      return c_nearest;
    };

    /**
     * @brief Get the configuration of the tree that is the closest to c,
     * using L1 distance between configurations.
     *
     * Linear in the size of the tree.
     *
     * @todo Complexity can be improved with space partitioning
     *
     * @param rand
     * @return std::shared_ptr<Configuration>
     */
    std::shared_ptr<Configuration> getNearestConfigurationByL1(
        const Configuration &c)
    {
      int mindist = -1;
      std::shared_ptr<Configuration> c_nearest = nullptr;

      for (auto v : this->getVertices())
      {
        int dist = 0;
        for (int agt = 0; agt < this->instance().nb_agents(); agt++)
        {
          auto cpos = this->instance().graph().movement().getPosition(c.at(agt));
          auto vpos = this->instance().graph().movement().getPosition((*v).at(agt));
          dist += (abs(vpos.first - cpos.first) +
                   abs(vpos.second - cpos.second));
        }
        if (dist < mindist || mindist == -1)
        {
          mindist = dist;
          c_nearest = v;
        }
      }
      assert(c_nearest);
      return c_nearest;
    }

    /**
     * @brief Get the vertex of the tree whose configuration is the closest to c,
     * using the sum of shortest path distances between configurations.
     *
     * Linear in the size of the tree.
     *
     * @todo Complexity can be improved with space partitioning
     *
     * @param c
     * @return std::shared_ptr<Configuration>
     */
    std::shared_ptr<Configuration> getNearestConfigurationBySPDistances(
        const Configuration &c)
    {
      int mindist = -1;
      std::shared_ptr<Configuration> c_nearest = nullptr;

      bool toGoal = c == this->instance().goal();
      std::vector<std::pair<int, std::shared_ptr<Configuration>>> distanceAndVertices;

      // if we are targeting goal, and if segment computations have become redundant, then pick a random one among nearest configurations,
      // rather than the nearest one
      if (toGoal && this->towards_goal_becoming_redundant_ ){
        for (auto v : this->getVertices())
        {
          // If the birdeye distance is more than mindist, then discard v: it cannot be the nearest
          int birdeye = 0;
          for (int agt = 0; agt < this->instance().nb_agents(); agt++)
          {
            birdeye += this->heuristics_.getBirdEyeDistance((*v).at(agt),c.at(agt));
          }
          distanceAndVertices.push_back(std::make_pair(birdeye, v));
        }
        std::sort(distanceAndVertices.begin(), distanceAndVertices.end());
        // Collect the <= 0.25 * this->vertices_.size() nearest configurations
        std::vector<std::shared_ptr<Configuration>> nearestVertices;
        int nmax = (int)((12+this->vertices_.size()) * 0.25); // so that this is at least 3
        int n = distanceAndVertices.size();
        if ( n >  nmax ) n = nmax;
        for(int i = 0; i < n; i++){
          // std::cout << "Collecting at distance " << distanceAndVertices[i].first << "\n";
          nearestVertices.push_back(distanceAndVertices[i].second);
        }
        // Pick a random one among these nearest configurations
        int irand = rand() % n;
        // std::cout << "Picked no " << irand << "\n";
        c_nearest = nearestVertices[irand];
        if (this->_verbose){
          std::cout << "* Picked randomly cnear: " << irand << " among " << n << "\n";
        }

        this->towards_goal_randomize_count_++;
        if (this->towards_goal_randomize_count_ >= this->towards_goal_randomize_period){
          this->towards_goal_randomize_count_ = 0;
          this->towards_goal_becoming_redundant_ = false;
        }
      } else {
        for (auto v : this->getVertices())
        {
          // If the birdeye distance is more than mindist, then discard v: it cannot be the nearest
          int birdeye = 0;
          for (int agt = 0; agt < this->instance().nb_agents(); agt++)
          {
            birdeye += this->heuristics_.getBirdEyeDistance((*v).at(agt),c.at(agt));
          }
          if (mindist >= 0 && birdeye >= mindist){
            continue;
          }
          // Otherwise compute the su mof shortest-paths distance
          int spdist = 0;
          for (int agt = 0; agt < this->instance().nb_agents(); agt++)
          {
            spdist += this->heuristics_.getShortestPathDistance((*v).at(agt),c.at(agt), true);
          }
          if (spdist < mindist || mindist == -1)
          {
            mindist = spdist;
            c_nearest = v;
          }
        }
      }
      assert(c_nearest);
      return c_nearest;
    }


    /**
     * @brief Return vector of neighbors of a given configuration at a given distance
     *
     * Complexity is linear in the size of the tree.
     *
     * @todo Complexity can be improved using space partitioning
     *
     * @param config
     * @return std::vector<std::shared_ptr<Configuration>>
     */
    std::vector<std::shared_ptr<Configuration>> getNeighborsAtDistance(
        const Configuration &config, int neardist)
    {
      std::vector<std::shared_ptr<Configuration>> neighbors;
      for (auto c : this->getVertices())
      {
        int dist = 0;
        for (int agt = 0; agt < this->instance().nb_agents(); agt++)
        {
          auto cpos = this->instance().graph().movement().getPosition(config.at(agt));
          auto vpos = this->instance().graph().movement().getPosition((*c).at(agt));
          dist += (abs(vpos.first - cpos.first) +
                   abs(vpos.second - cpos.second));
        }
        if (dist < neardist)
        {
          neighbors.push_back(c);
        }
      }
      return neighbors;
    };


    /**
     * @brief sort the configurations
     *
     * @param 
     */
    bool sortConfiguration(
        const Configuration &first,
        const Configuration &second)
    {
      auto firstpath = getExecution(this->instance().start(), first);
      auto secondpath = getExecution(this->instance().start(), second);
      return (firstpath.size() <= secondpath.size());
    }



    std::vector<std::shared_ptr<Configuration>> &getVertices()
    {
      return vertices_;
    }

    std::vector<std::shared_ptr<Configuration>> &getParents()
    {
      return parents_;
    }

    const Instance<GraphMove, GraphComm> &instance()
    {
      return this->instance_;
    }

    bool treeContains(const Configuration &config)
    {
      return index_of_vertex_.find(config) != index_of_vertex_.end();
    }

    void addConfiguration(std::shared_ptr<Configuration> c_new, std::shared_ptr<Configuration> c_parent, std::vector<std::shared_ptr<Configuration>> pathSegment){
        this->vertices_.push_back(c_new);
        this->parents_.push_back(c_parent);
        this->index_of_vertex_[*c_new] = this->vertices_.size() - 1;
        this->index_of_parent_[*c_new] = this->index_of_vertex_[*c_parent];
        this->path_from_parent_[*c_new] = pathSegment;
        if (_verbose) {
          std::cout << "Adding Cnew: " << ANSI_YELLOW << *c_new << ANSI_RESET << "\n";
        }
    }

    /**
     * @brief Extend the tree by a new node: select a random target configuration, compute a bounded path towards,
     * and connect the last configuration of this path to the tree.
     * 
     */
    void expand()
    {
      clock_t cstart;
      clock_t cend;

      // Random configuration serving as random direction
      std::shared_ptr<Configuration> c_rand =
          getRandomConfiguration(this->instance().goal());

      // c_target is the target configuration to expand the tree: it is (randomly) either c_rand, or the goal configuration
      std::shared_ptr<Configuration> c_target = c_rand;
      int irand = rand() % 100;
      if (irand <= this->_prob2goal)
      {
        c_target = std::make_shared<Configuration>(instance().goal());
      }

      if (_verbose){
        std::cout << "\nTree size: " << vertices_.size() << "\n";
        if (*c_target == this->instance().goal())
        {
          std::cout << ANSI_PURPLE << "Moving towards " << ANSI_BOLD << "GOAL: " << ANSI_RESET << (*c_target) << "\n";
        }
        else
        {
          std::cout << ANSI_CYAN << "Moving towards " << ANSI_BOLD << "RANDOM: " << ANSI_RESET << (*c_target) << "\n";
        }
        std::cout.flush();
      }

      cstart = clock();
      std::shared_ptr<Configuration> c_nearest = getNearestConfigurationBySPDistances(*c_target);
      cend = clock();
      if (_verbose) {
        std::cout << "\tNearest node in the tree: " << ANSI_YELLOW << *c_nearest << ANSI_RESET << "\n";
        // std::cout << "\tComputed in " << ANSI_RED << (cend - cstart) / (double) CLOCKS_PER_SEC << "s\n" << ANSI_RESET;
        std::cout.flush();
      }
      
      std::vector<std::shared_ptr<Configuration>> pathSegment;
      BoundedSolver<GraphMove, GraphComm> *currentSubsolver = nullptr;

      switch (this->subsolver)
      {
      case SubsolverEnum::COORD:
        currentSubsolver = (BoundedSolver<GraphMove, GraphComm> *)&coord_solver_;
        break;
      case SubsolverEnum::DFS:
        currentSubsolver = (BoundedSolver<GraphMove, GraphComm> *)&dfs_solver_;
        break;
      }
      assert(currentSubsolver);
      cstart = clock();



      // If we are moving towards a random configuration; then limit the nb of iterations
      pathSegment = currentSubsolver->computeBoundedPathTowards(*c_nearest, *c_target, this->_step_size, this->max_subsolver_iterations_);

      // Now decide if we want to run DFS instead
      bool use_dfs = false;
      int use_dfs_max_iterations = -1;
      if (this->subsolver == SubsolverEnum::COORD && !this->_window_connected && *c_target == this->instance().goal())
      {
        use_dfs = true;
        size_t cnear_sp_dist = 0;
        size_t cnear_be_dist = 0;
        size_t min_dist = INFINITY;
        size_t max_dist = 0;
        int xmax = 0;
        int xmin = INFINITY;
        int ymax = 0;
        int ymin = INFINITY;

        // We will use dfs except in the following cases:
        // If one of the agents is far from its target, do not use DFS
        for (int agt = 0; agt < instance().nb_agents(); agt++)
        {
          size_t agt_sp_dist = this->heuristics_.getShortestPathDistance(c_nearest->at(agt), c_target->at(agt));
          cnear_sp_dist += agt_sp_dist;
          if (agt_sp_dist < min_dist ){
            min_dist = agt_sp_dist;
          }
          if (agt_sp_dist > max_dist){
            max_dist = agt_sp_dist;
          }
          cnear_be_dist += this->heuristics_.getBirdEyeDistance(c_nearest->at(agt), c_target->at(agt));
          std::pair<int,int> pos = this->instance().graph().movement().getPosition(c_nearest->at(agt));
          if (pos.first < xmin){
            xmin = pos.first;
          }
          if (pos.first > xmax){
            xmax = pos.first;
          }
          if (pos.second > ymax){
            ymax = pos.second;
          }
          if (pos.second < ymin){
            ymin = pos.second;
          }
        }
        // If COORD solver got us closer to goal (than cnear), then no need to switch to DFS solver
        // if (cnear_sp_dist >= cnear_be_dist * 1.2 || min_dist > 8 ){
        // std::pair<float, float> cnear_barycenter = this->instance().graph().movement().getBarycenter(*c_nearest);
        // std::pair<float, float> cgoal_barycenter = this->instance().graph().movement().getBarycenter(*c_target);
        // float xdiff = abs(cnear_barycenter.first - cgoal_barycenter.first);
        // float ydiff = abs(cnear_barycenter.second - cgoal_barycenter.second);
        size_t stretch_threshold = this->instance().nb_agents() / 1.5;
        if (stretch_threshold < 5) 
          stretch_threshold = 5;
        if (cnear_sp_dist >= cnear_be_dist * 1.2 
            || min_dist > 5 
            // || xdiff > 10 || ydiff > 10 
            || (xmax -xmin)> stretch_threshold 
            || (ymax-ymin) > stretch_threshold)
        {
          if (this->_verbose) {
            std::cout << "* Not switching to DF: cnear_sp_dist / cnear_be_dist = " << cnear_sp_dist / (double) cnear_be_dist << " \n";
            std::cout << "* Min/Max Distance of agents to their goals: Min=" << ANSI_RED << min_dist << ANSI_RESET << ", max=" << ANSI_RED <<max_dist << ANSI_RESET << "\n";
            // std::cout << "* Dist between barycenters (cnear and goal): xdiff=" << xdiff << " ydiff=" << ydiff << "\n";
            std::cout << "* Ecartement. x=" << ANSI_RED << (xmax-xmin) << ANSI_RESET  << " y=" << ANSI_RESET << (ymax-ymin)  << ANSI_RESET << "\n";
          }
          use_dfs = false;
        } else if (this->_verbose){
          std::cout << "* Switching to dfs: " << "cnear_sp_dist / cnear_be_dist = " << (cnear_sp_dist / (double) cnear_be_dist) << " <= 1.2\n";
            std::cout << "* Min/Max Distance of agents to their goals: Min=" << min_dist << ", max=" << max_dist << "\n";
            // std::cout << "* Dist between barycenters (cnear and goal): xdiff=" << xdiff << " ydiff=" << ydiff << "\n";
            std::cout << "* Ecartement. x=" << ANSI_RED << (xmax-xmin)  << ANSI_RESET << " y=" << ANSI_RED << (ymax-ymin)  << ANSI_RESET << "\n";
        }

        // If instance is not window connected, and if we are targeting goal,
        //    and if we are almost straight-line to goal
        //    (if all shortest paths are close to birdeye distance)
        //    and if coord_solver did not get us closer to goal
        //    then use dfs_solver      
        if (pathSegment.size() > 0)
        {
          auto coord_cnew = pathSegment.back();
          size_t coordcnew_sp_dist = 0;
          for (int agt = 0; agt < instance().nb_agents(); agt++)
          {
            coordcnew_sp_dist += this->heuristics_.getShortestPathDistance(coord_cnew->at(agt), c_target->at(agt));
          }

          // If COORD solver got us closer to goal (than cnear), then no need to switch to DFS solver
          if (coordcnew_sp_dist < this->_min_distance ){
            if (this->_verbose) {
              std::cout << "* Not switching to DFS: Coord is improving...\n";
              std::cout << "* SP(coordcnew) = " 
                << ANSI_BLUE << coordcnew_sp_dist << ANSI_RESET << " < min_distance = " << this->_min_distance << "\n";
              std::cout.flush();
            }
            this->_min_distance = coordcnew_sp_dist;
            use_dfs = false;
          }
        } else {
          if (_verbose) {
            std::cout << ANSI_RED << "Subsolver failed. Falling back to dfs_solver.\n"
                      << ANSI_RESET;
          }
          use_dfs = true;
          use_dfs_max_iterations = this->max_subsolver_iterations_;
        }
      }
      if (use_dfs){
        if (pathSegment.size() >0){
          auto c_new = pathSegment.back();
          if (not(this->treeContains(*c_new)))
          {
            addConfiguration(c_new, c_nearest, pathSegment);          
          }
        }
        std::cout << ANSI_BOLD << ANSI_BLUE << "Switching to dfs_solver.\n" << ANSI_RESET;
        std::cout.flush();
        pathSegment = dfs_solver_.computeBoundedPathTowards(*c_nearest, *c_target, this->_step_size,use_dfs_max_iterations);
        std::cout << "Done: " << pathSegment.size() << "\n";
      }
      cend = clock();
      if (_verbose){
        if (((cend - cstart) / (double)CLOCKS_PER_SEC > 0.1) || _verbose)
        {
          std::cout << "Subsolver took " << ANSI_RED << (cend - cstart) / (double)CLOCKS_PER_SEC << ANSI_RESET << " seconds\n"
                    ;
        } else {
          std::cout << "Subsolver took " << ANSI_BLUE << (cend - cstart) / (double)CLOCKS_PER_SEC << ANSI_RESET << " seconds\n";
        }
      }
      if (pathSegment.size() == 0)
      {
        if (_verbose){
          std::cout << ANSI_RED << "computeBoundedPathTowards failed.\n"
                    << ANSI_RESET;
          std::cout.flush();
        }
        return;
      }

      std::shared_ptr<Configuration> c_new = pathSegment.back();
      if (not(this->treeContains(*c_new)))
      {
        addConfiguration(c_new, c_nearest, pathSegment);
      } else {
        if (!this->towards_goal_becoming_redundant_)
          this->towards_goal_becoming_redundant_ = true;
        std::cout << ANSI_RED << "Cnew was already in the tree : " << *c_new << ANSI_RESET << " (use_dfs: " << use_dfs << ")\n";
      }

      if (_cmarrtstar){
        auto source = this->instance().start();
        int neardist = this->_step_size/2;
        std::shared_ptr<Configuration> c_min = c_nearest;
        auto pathSegmentMin = currentSubsolver->computeBoundedPathTowards(*c_min, *c_new, this->_step_size);
        int costmin = getExecution(source, *c_new).size();
        std::vector<std::shared_ptr<Configuration>> Neighborhood =
           getNeighborsAtDistance(*c_new, neardist);
        //std::cout << "Neigbourhood size " << Neighborhood.size() << "\n";
        std::sort(Neighborhood.begin(), Neighborhood.end(), 
                [this](std::shared_ptr<Configuration> a, std::shared_ptr<Configuration> b) {
                    return this->sortConfiguration(*a,*b);} );
        //std::cout << "Neigbourhood size " << Neighborhood.size() << "\n";
        for (int k = 0; k < 3; k++) {
          if (k < Neighborhood.size()) {
            //std::cout << "new cnear\n";
            auto cnear = Neighborhood.at(k);
            auto pathSegmentNear = coord_solver_.computeBoundedPathTowards(*cnear, *c_new, this->_step_size);
            int cost = getExecution(source, *cnear).size() + pathSegmentNear.size();
            if (cost < costmin && pathSegmentNear.size() > 0) {
              c_min = cnear;
              costmin = cost;
              pathSegmentMin = pathSegmentNear;
            }
          }
        }

        this->parents_[index_of_vertex_[*c_new]] = std::make_shared<Configuration>(*c_min); 
        this->index_of_parent_[*c_new] = index_of_vertex_[*c_min];
        this->path_from_parent_[*c_new] = pathSegment;
      }
    }

    /**
     * @brief Compute full execution from configuration first to configuration second
     * @pre first and second both belong to the tree
     * @param first
     * @param second
     * @return std::vector<std::shared_ptr<Configuration>>
     */
    std::vector<std::shared_ptr<Configuration>> getExecution(
        const Configuration &first, const Configuration &second)
    {
      assert(treeContains(first));
      assert(treeContains(second));
      std::vector<std::shared_ptr<Configuration>> exec;
      auto c = std::make_shared<Configuration>(second);
      while (index_of_vertex_.find(*c) != index_of_vertex_.find(first))
      {
        std::vector<std::shared_ptr<Configuration>> segment = this->path_from_parent_[*c];
        exec.insert(exec.begin(), segment.begin(), (segment.end()));
        c = this->vertices_[this->index_of_parent_[*c]];
      }
      return exec;
    }


    void printTree()
    {
      int node_count = this->vertices_.size();
      std::shared_ptr<bool[]> visited(new bool[node_count]);
      for (int i = 0; i < node_count; i++)
      {
        visited[i] = false;
      }
      for (int v = 0; v < node_count; v++)
      {
        if (visited[v])
        {
          continue;
        }
        while (!visited[v])
        {
          visited[v] = true;
          int parent_of_v = this->index_of_parent_[*(this->vertices_[v])];
          if (v != parent_of_v)
          {
            std::cout << "RRT edge: " << (*this->vertices_[v]) << " -- " << (*this->vertices_[parent_of_v]) << "\n";
          }
          v = parent_of_v;
        }
      }
    }

  public:
    /**
     * @brief Construct a new Exploration Tree that initially contains the
     * starting configuration of the instance.
     *
     * @param instance
     * @param objective
     * @param _prob2goal probability of picking goal configuration at a given iteration as a direction to expand the tree
     * @param step_size the number of steps the tree is expanded towards a given direction
     */
    CMARRT(const Instance<GraphMove, GraphComm> &instance,
           const Objective &objective,
           Heuristics<GraphMove, GraphComm> &heuristics,
           SubsolverEnum subsolver,
           int _prob2goal,
           int step_size,
           int window_size,
           bool cmarrtstar,
           bool verbose=false)
        : Solver<GraphMove, GraphComm>(instance, objective),
          heuristics_(heuristics),
          _prob2goal(_prob2goal),
          _step_size(step_size),
          _cmarrtstar(cmarrtstar),
          _verbose(verbose),
          subsolver(subsolver),
          dfs_solver_(instance, objective, heuristics, false),
          coord_solver_(instance, objective, heuristics, window_size)
    {
      auto start = std::make_shared<Configuration>(instance.start());
      vertices_.push_back(start);
      parents_.push_back(start);
      this->_window_connected = 
        coordinated::CoordSolver<ExplicitGraph, ExplicitGraph>::isConfigurationWindowConnected(instance.start(), instance, window_size)
        && 
        coordinated::CoordSolver<ExplicitGraph, ExplicitGraph>::isConfigurationWindowConnected(instance.goal(), instance, window_size);
      this->_min_distance = INFINITY;
    }

    bool step_compute()
    {
      if (this->treeContains(this->instance().goal()))
      {
        std::cout << "Done after " << _iterations << " iterations\n";
        std::cout.flush();
        auto exec = this->getExecution(this->instance().start(), this->instance().goal());

        // Check if the configurations are connected
        for (size_t i = 0; i < exec.size(); i++){
          assert(instance().graph().communication().isConfigurationConnected(*exec[i]));
          if (instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS){
            assert(!exec[i]->hasCollisions());
          }
        }
        this->execution_.setPaths(exec);

        for (size_t agt = 0; agt < this->instance_.nb_agents(); agt++){
          assert(this->execution_.getPath(agt)->isValid(instance().graph().movement()));
        }
        return true;
      }
      else
      {
        if (_verbose){
          std::cout << "\n#Iterations: " << _iterations << "\n";
        }
        _iterations++;
        this->expand();
        if (_iterations % 10 == 0 && _verbose)
        {
          this->printTree();
        }
        return false;
      }
    }

    virtual const Execution compute() override
    {
      while(!step_compute());
      return this->execution_;
    }

  };


} // namespace cmarrt