#pragma once

#include <CTNOrderingStrategy.hpp>
#include <ConflictSelectionStrategy.hpp>
#include <ConstraintTreeNode.hpp>
#include <DFS.hpp>
#include <ShortestPathCalculator.hpp>
#include <Instance.hpp>
#include <LowLevel.hpp>
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
    bool _rrtstar = false;
    // Whether start and goal configurations are window-connected
    bool _window_connected = false;

    Heuristics<GraphMove, GraphComm> &heuristics_;

    SubsolverEnum subsolver;

    std::vector<std::shared_ptr<Configuration>> vertices_;
    std::vector<std::shared_ptr<Configuration>> parents_;
    std::map<Configuration, int> index_of_vertex_;
    std::map<Configuration, int> index_of_parent_;
    // given configuration c in the tree, path_from_parent_[c] is the path from the parent to itself
    std::map<Configuration, std::vector<std::shared_ptr<Configuration>>> path_from_parent_;

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
          configuration->PushBack(config.at(agt));
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

      for (auto v : this->getVertices())
      {
        int dist = 0;
        for (int agt = 0; agt < this->instance().nb_agents(); agt++)
        {
          // auto cpos = this->instance().graph().movement().getPosition(c.at(agt));
          // auto vpos = this->instance().graph().movement().getPosition((*v).at(agt));
          // dist += (abs(vpos.first - cpos.first) +
          //          abs(vpos.second - cpos.second));
          dist += this->heuristics_.getShortestPathDistance(c.at(agt), (*v).at(agt));
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
     * @brief Return vector of neighbors of a given configuration at a given distance
     * (comparing L1 distances between barycenters).
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

    bool isPathSegmentValid(std::vector<std::shared_ptr<Configuration>> pathSegment){
      return true;
      /*
      for (size_t i = 1; i < pathSegment.size(); i++){
        Node next_n = at(i);
        const std::unordered_set<Node>& neighbors = graph.get_neighbors(n);
        if (neighbors.find(next_n) == neighbors.end()){
          std::cerr << "The following path is not valid\n" << *this << "\n";
          return false;
        }
        n = next_n;
      }
      return true;
      */
    }
    /**
     * @brief Extend the tree by a new node: select a random target configuration, compute a bounded path towards,
     * and connect the last configuration of this path to the tree.
     * 
     */
    void extend()
    {
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
      }

      std::shared_ptr<Configuration> c_nearest = getNearestConfigurationBySPDistances(*c_target);

      if (_verbose) {
        std::cout << "\tNearest node in the tree: " << ANSI_YELLOW << *c_nearest << ANSI_RESET << "\n";
        std::cout.flush();
      }
      
      // If c_target == goal, then randomly (1/3) flip the direction to c_rand
      // In this case, we still go from c_nearest (which is neareast to goal), but go in a random direction
      if (*c_target == this->instance().goal() && rand()%3 == 0){
        c_target = c_rand;
        if (_verbose){
          std::cout << "<->" << ANSI_BLUE << "\tRandom direction from nearest to goal " << ANSI_RESET << "\n";
        }
      }
      std::vector<std::shared_ptr<Configuration>> pathSegment;
      BoundedSolver<GraphMove, GraphComm> *currentSubsolver = nullptr;

      switch (this->subsolver)
      {
      case SubsolverEnum::COORD_SOLVER:
        currentSubsolver = (BoundedSolver<GraphMove, GraphComm> *)&coord_solver_;
        // If instance is not window connected, and if we are targeting goal,
        //    and if we are almost straight-line to goal
        //    (if all shortest paths are close to birdeye distance), then use dfs_solver
        
        if (!this->_window_connected && *c_target == this->instance().goal())
        {
          bool use_dfs = true;
          for (int agt = 0; agt < instance().nb_agents(); agt++)
          {
            if (this->heuristics_.getShortestPathDistance(c_nearest->at(agt), c_target->at(agt)) >
                this->heuristics_.getBirdEyeDistance(c_nearest->at(agt), c_target->at(agt)) + 1)
            {
              use_dfs = false;
              break;
            }
          }
          if (use_dfs)
          {
            if (_verbose) {
              std::cout << ANSI_BOLD << ANSI_BLUE << "We are near goal: using dfs_solver.\n" << ANSI_RESET;}
              currentSubsolver = (BoundedSolver<GraphMove, GraphComm> *)&dfs_solver_;
            }
        }
        break;
      case SubsolverEnum::DFS_SOLVER:
        currentSubsolver = (BoundedSolver<GraphMove, GraphComm> *)&dfs_solver_;
        break;
      }
      assert(currentSubsolver);
      auto cstart = clock();
      pathSegment = currentSubsolver->computeBoundedPathTowards(*c_nearest, *c_target, this->_step_size);

      // In case of failure, try again with dfs_solver
      if (this->subsolver == SubsolverEnum::COORD_SOLVER && pathSegment.size() == 0 ||
          this->subsolver == SubsolverEnum::DECOUPLED_SOLVER && pathSegment.size() < this->_step_size / 2)
      {
        if (_verbose) {
          std::cout << ANSI_RED << "Subsolver failed. Falling back to dfs_solver.\n"
                    << ANSI_RESET;
          this->printTree();
        }
        pathSegment = dfs_solver_.computeBoundedPathTowards(*c_nearest, *c_target, this->_step_size);
      }
      auto cend = clock();
      if (_verbose){
        if (((cend - cstart) / (double)CLOCKS_PER_SEC > 0.1) && _verbose)
        {
          std::cout << ANSI_RED << "Subsolver took " << (cend - cstart) / (double)CLOCKS_PER_SEC << " seconds\n"
                    << ANSI_RESET;
        } else {
          std::cout << "Subsolver took " << ANSI_BLUE << (cend - cstart) / (double)CLOCKS_PER_SEC << ANSI_RESET << " seconds\n";
        }
      }
      if (pathSegment.size() == 0)
      {
        if (_verbose){
        std::cout << ANSI_RED << "computeBoundedPathTowards failed; ignoring this step.\n"
                  << ANSI_RESET;}
        std::cout.flush();
        return;
      }

      std::shared_ptr<Configuration> c_new = pathSegment.back();
      if (not(this->treeContains(*c_new)))
      {
        this->vertices_.push_back(c_new);
        this->parents_.push_back(c_nearest);
        this->index_of_vertex_[*c_new] = this->vertices_.size() - 1;
        this->index_of_parent_[*c_new] = this->index_of_vertex_[*c_nearest];
        this->path_from_parent_[*c_new] = pathSegment;
        if (_verbose) {
          std::cout << "Adding Cnew: " << ANSI_YELLOW << *c_new << ANSI_RESET << "\n";
        }
      } else if (_verbose)
      {
        std::cout << ANSI_RED << "Cnew was already in the tree : " << *c_new << ANSI_RESET << "\n";
        std::cout << "\tPath segment has size " << pathSegment.size() << ": \n";
        for (auto c : pathSegment){
          std::cout << "\t" << *c;
          std::cout << "\n";
        }
      }

      if (_rrtstar){
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

    
    /**
     * @brief Compute full execution from start configuration to given goal configuration
     * @pre goal belongs to the tree
     * @param goal
     * @return std::vector<std::shared_ptr<Configuration>>
     */
    std::vector<std::shared_ptr<Configuration>> getExecution(
        const Configuration &goal)
    {
      return getExecution(instance().start(), goal);
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
           bool verbose=false)
        : Solver<GraphMove, GraphComm>(instance, objective),
          heuristics_(heuristics),
          _prob2goal(_prob2goal),
          _step_size(step_size),
          _verbose(verbose),
          subsolver(subsolver),
          dfs_solver_(instance, objective, heuristics),
          coord_solver_(instance, objective, heuristics, window_size)
    {
      auto start = std::make_shared<Configuration>(instance.start());
      vertices_.push_back(start);
      parents_.push_back(start);
      this->_window_connected = 
        coordinated::CoordSolver<ExplicitGraph, ExplicitGraph>::isConfigurationWindowConnected(instance.start(), instance, window_size)
        && 
        coordinated::CoordSolver<ExplicitGraph, ExplicitGraph>::isConfigurationWindowConnected(instance.goal(), instance, window_size);
    }

    bool StepCompute() override
    {
      if (this->treeContains(this->instance().goal()))
      {
        //this->printTree();
        std::cout << "Done after " << _iterations << " iterations\n";
        std::cout.flush();
        auto exec = this->getExecution(this->instance().goal());
        // Check if the configurations are connected
        for (size_t i = 0; i < exec.size(); i++){
          assert(instance().graph().communication().isConfigurationConnected(*exec[i]));
          if (instance().getCollisionMode() == CollisionMode::CHECK_COLLISIONS){
            assert(!exec[i]->hasCollisions());
          }
        }
        for (size_t agt = 0; agt < this->instance_.nb_agents(); agt++)
        {
          std::shared_ptr<Path> p_agt = std::make_shared<Path>();
          for (size_t t = 0; t < exec.size(); t++)
          {
            p_agt->PushBack(exec.at(t)->at(agt));
          }
          assert(p_agt->isValid(instance().graph().movement()));
          this->execution_.set_path(agt, p_agt);
        }
        return true;
      }
      else
      {
        if (_verbose){
          std::cout << "\n#Iterations: " << _iterations << "\n";
        }
        _iterations++;
        this->extend();
        if (_iterations % 10 == 0 && _verbose)
        {
          this->printTree();
        }
        return false;
      }
    }
  };

} // namespace cmarrt