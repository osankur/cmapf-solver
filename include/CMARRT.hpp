#pragma once

#include <CTNOrderingStrategy.hpp>
#include <ConflictSelectionStrategy.hpp>
#include <ConstraintTreeNode.hpp>
#include <DFS.hpp>
#include <FloydWarshall.hpp>
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
   * @class CMARRT for the algorithm Connected Multi-agent RRT (Rapidly-exploring
   * random tree)
   *
   * @todo Make CMARRT inherit from ExplorationTree or just merge the two classes
   * */
  template <class GraphMove, class GraphComm>
  class CMARRT : public Solver<GraphMove, GraphComm>
  {
  private:
    int iterations = 0;
    int prob2target; // bias in % to use the target configuration for the random one
    int step_size;
    bool debug = true;
    bool rrtstar = false;

    Heuristics<GraphMove, GraphComm> &heuristics_;

    SubsolverEnum subsolver;

    std::vector<std::shared_ptr<Configuration>> vertices_;
    // FIXME is parents_[i] the parent node of vertices_[i]?
    std::vector<std::shared_ptr<Configuration>> parents_;
    std::map<Configuration, int> index_of_vertex_;
    std::map<Configuration, int> index_of_parent_;
    // given configuration c in the tree, path_from_parent_[c] is the path from the parent to itself
    std::map<Configuration, std::vector<std::shared_ptr<Configuration>>> path_from_parent_;

    coupled::DFS<GraphMove, GraphComm> dfs_solver_;
    decoupled::BoundedDecoupledSolver<GraphMove, GraphComm> decoupled_solver_;
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
     * @brief returns the random configuration (used then to compute the nearest
     * configuration)
     *
     * @param goal
     * @return std::shared_ptr<Configuration>
     */
    std::shared_ptr<Configuration> getRandomConfiguration(
        const Configuration &goal)
    {
      int irand = rand() % 100;
      if (irand > this->prob2target)
      {

        // Pick a node of Gc for the 1st agent
        size_t nb_nodes = this->instance().graph().communication().node_count();
        Node first = (uint64_t)rand() % (nb_nodes - 1);

        std::vector<Node> config = {first};
        // config.push_back(first);
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
      }
      else
      {
        return std::make_shared<Configuration>(goal);
      }
    };

    /**
     * @brief Get the nearest configuration in the tree in the direction given
     * by the configuration rand. We use the barycenter for evaluating the
     * distance from an existing configuration to rand.
     *
     * Linear in the size of the tree.
     *
     * @todo Complexity can be improved with space partitioning
     *
     * @param rand
     * @return std::shared_ptr<Configuration>
     */
    std::shared_ptr<Configuration> getNearestConfigurationByBarycenter(
        const Configuration &rand)
    {
      int mindist = -1;
      std::shared_ptr<Configuration> c_nearest = nullptr;
      std::pair<float, float> rand_pos = this->instance_.graph().movement().getBarycenter(rand);
      for (auto v : this->getVertices())
      {
        // distance v->rand
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
     * @brief Get the nearest configuration in the tree in the direction given
     * by the configuration rand. The distance is computed with the l1 metric.
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
    };

    /**
     * @brief compute the neighbors of the configuration in the tree
     *
     * Complexity is linear in the size of the tree.
     *
     * @todo Complexity can be improved using space partitioning
     *
     * @param config
     * @return std::vector<std::shared_ptr<Configuration>>
     */
    std::vector<std::shared_ptr<Configuration>> NeighborsAtDistance(
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

    void extend()
    {
      std::shared_ptr<Configuration> c_rand =
          getRandomConfiguration(this->instance().goal());

      if (*c_rand == this->instance().goal() & debug)
      {
        std::cout << ANSI_PURPLE << "Moving towards " << ANSI_BOLD << "GOAL: " << ANSI_RESET << (*c_rand) << "\n";
      }
      else if (debug)
      {
        std::cout << ANSI_CYAN << "Moving towards " << ANSI_BOLD << "RANDOM: " << ANSI_RESET << (*c_rand) << "\n";
      }

      std::shared_ptr<Configuration> c_nearest =
          getNearestConfigurationByL1(*c_rand);

      if (debug) {
      std::cout << "Cnearest is " << ANSI_YELLOW << *c_nearest << ANSI_RESET << "\n";}
      std::cout.flush();

      std::vector<std::shared_ptr<Configuration>> pathSegment;
      BoundedSolver<GraphMove, GraphComm> *currentSubsolver = nullptr;

      switch (this->subsolver)
      {
      case SubsolverEnum::DECOUPLED_SOLVER:
        currentSubsolver = (BoundedSolver<GraphMove, GraphComm> *)&decoupled_solver_;
        break;
      case SubsolverEnum::COORD_SOLVER:
        currentSubsolver = (BoundedSolver<GraphMove, GraphComm> *)&coord_solver_;
        // if (*c_rand == this->instance().goal())
        // {
        //   // If all shortestpaths are close to birdeye distances, then use dfs_solver
        //   bool use_dfs = true;
        //   for (int agt = 0; agt < instance().nb_agents(); agt++)
        //   {
        //     if (this->heuristics_.getShortestPathDistance(c_nearest->at(agt), c_rand->at(agt)) >
        //         this->heuristics_.getBirdEyeDistance(c_nearest->at(agt), c_rand->at(agt)) + 1)
        //     {
        //       use_dfs = false;
        //       break;
        //     }
        //   }
        //   if (use_dfs)
        //   {
        //     if (debug) {
        //     std::cout << ANSI_BOLD << ANSI_BLUE << "Near target: using dfs_solver.\n" << ANSI_RESET;}
        //     currentSubsolver = (BoundedSolver<GraphMove, GraphComm> *)&dfs_solver_;
        //   }
        // }
        break;
      case SubsolverEnum::DFS_SOLVER:
        currentSubsolver = (BoundedSolver<GraphMove, GraphComm> *)&dfs_solver_;
        break;
      }
      assert(currentSubsolver != nullptr);
      auto cstart = clock();
      pathSegment = currentSubsolver->computeBoundedPathTowards(*c_nearest, *c_rand, this->step_size);

      // In case of failure, try again with dfs_solver
      if (this->subsolver == SubsolverEnum::COORD_SOLVER && pathSegment.size() == 0 ||
          this->subsolver == SubsolverEnum::DECOUPLED_SOLVER && pathSegment.size() < this->step_size / 2)
      {
        if (debug) {
        std::cout << ANSI_RED << "Subsolver failed. Falling back to dfs_solver.\n"
                  << ANSI_RESET;}
        pathSegment = dfs_solver_.computeBoundedPathTowards(*c_nearest, *c_rand, this->step_size);
      }
      auto cend = clock();
      if (((cend - cstart) / (double)CLOCKS_PER_SEC > 0.1) && debug)
      {
        std::cout << ANSI_RED << "Subsolver took " << (cend - cstart) / (double)CLOCKS_PER_SEC << " seconds\n"
                  << ANSI_RESET;
      }

      if (pathSegment.size() == 0)
      {
        if (debug){
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
        if (debug) {
        std::cout << "Adding Cnew: " << ANSI_YELLOW << *c_new << ANSI_RESET << "\n";}
      }
      else if (debug)
      {
        std::cout << ANSI_RED << "Cnew is already in the tree : " << *c_new << ANSI_RESET << "\n";
      }

      if (rrtstar){
        auto source = this->instance().start();
        int neardist = this->step_size/2;
        std::shared_ptr<Configuration> c_min = c_nearest;
        auto pathSegmentMin = currentSubsolver->computeBoundedPathTowards(*c_min, *c_new, this->step_size);
        int costmin = getExecution(source, *c_new).size();
        std::vector<std::shared_ptr<Configuration>> Neighborhood =
           NeighborsAtDistance(*c_new, neardist);
        //std::cout << "Neigbourhood size " << Neighborhood.size() << "\n";
        std::sort(Neighborhood.begin(), Neighborhood.end(), 
                [this](std::shared_ptr<Configuration> a, std::shared_ptr<Configuration> b) {
                    return this->sortConfiguration(*a,*b);} );
        //std::cout << "Neigbourhood size " << Neighborhood.size() << "\n";
        for (int k = 0; k < 3; k++) {
          if (k < Neighborhood.size()) {
            //std::cout << "new cnear\n";
            auto cnear = Neighborhood.at(k);
            auto pathSegmentNear = coord_solver_.computeBoundedPathTowards(*cnear, *c_new, this->step_size);
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
     * @brief Given an ExplorationTree, compute the from a first configuration to a second
     * @pre first and second belongs to the tree
     * @param first
     * @param second
     * @return std::vector<std::shared_ptr<Configuration>>
     */
    std::vector<std::shared_ptr<Configuration>> getExecution(
        const Configuration &first, const Configuration &second)
    {
      std::vector<std::shared_ptr<Configuration>> exec;
      auto c = std::make_shared<Configuration>(second);
      while (index_of_vertex_.find(*c) != index_of_vertex_.find(first))
      {
        std::vector<std::shared_ptr<Configuration>> segment = this->path_from_parent_[*c];
        exec.insert(exec.begin(), segment.begin(), (segment.end() - 1));
        c = this->vertices_[this->index_of_parent_[*c]];
      }
      return exec;
    }

    
    /**
     * @brief Given an ExplorationTree, compute an execution from start to goal
     * @pre goal belongs to the tree
     * @param start
     * @param goal
     * @return std::vector<std::shared_ptr<Configuration>>
     */
    std::vector<std::shared_ptr<Configuration>> getExecution(
        const Configuration &goal)
    {
      std::vector<std::shared_ptr<Configuration>> exec;
      auto c = std::make_shared<Configuration>(goal);
      while (this->path_from_parent_.count(*c) > 0)
      {
        std::vector<std::shared_ptr<Configuration>> segment = this->path_from_parent_[*c];
        exec.insert(exec.begin(), segment.begin(), (segment.end() - 1));
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
     * @param prob2target probability of picking goal configuration at a given iteration as a direction to expand the tree
     * @param step_size the number of steps the tree is expanded towards a given direction
     */
    CMARRT(const Instance<GraphMove, GraphComm> &instance,
           const Objective &objective,
           Heuristics<GraphMove, GraphComm> &heuristics,
           std::shared_ptr<FloydWarshall<GraphMove, GraphComm>> &fw,
           SubsolverEnum subsolver,
           int prob2target,
           int step_size,
           int window_size)
        : Solver<GraphMove, GraphComm>(instance, objective),
          heuristics_(heuristics),
          prob2target(prob2target),
          step_size(step_size),
          subsolver(subsolver),
          dfs_solver_(instance, objective, heuristics),
          decoupled_solver_(instance, objective, fw),
          coord_solver_(instance, objective, heuristics, window_size)
    {
      auto start = std::make_shared<Configuration>(instance.start());
      vertices_.push_back(start);
      parents_.push_back(start);
      // step and neardist
      // int size = instance.graph().movement().node_count();
      // step = size / 30;
      // neardist = size / 50;
    }

    bool StepCompute() override
    {
      if (debug)
        std::cout << "\nStep \n";
      if (this->treeContains(this->instance().goal()))
      {
        //this->printTree();
        std::cout << "Done after " << iterations << " iterations\n";
        std::cout.flush();
        //auto exec = this->getExecution(this->instance().start(),this->instance().goal());
        auto exec = this->getExecution(this->instance().goal());
        for (size_t agt = 0; agt < this->instance_.nb_agents(); agt++)
        {
          std::shared_ptr<Path> p_agt = std::make_shared<Path>();
          for (size_t t = 0; t < exec.size(); t++)
          {
            p_agt->PushBack(exec.at(t)->at(agt));
          }
          this->execution_.set_path(agt, p_agt);
        }
        return true;
      }
      else
      {
        //std::cout << "\n#Iterations: " << iterations << "\n";
        iterations++;
        this->extend();
        if (iterations % 10 == 0 && debug)
        {
          this->printTree();
        }
        return false;
      }
    }
  };

} // namespace cmarrt