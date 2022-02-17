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

#define ANSI_RESET  "\u001B[0m"
#define ANSI_BLACK  "\u001B[30m"
#define ANSI_RED  "\u001B[31m"
#define ANSI_GREEN  "\u001B[32m"
#define ANSI_YELLOW  "\u001B[33m"
#define ANSI_BLUE  "\u001B[34m"
#define ANSI_PURPLE  "\u001B[35m"
#define ANSI_CYAN  "\u001B[36m"
#define ANSI_WHITE  "\u001B[37m"


namespace cmarrt
{

  /**
   * @class CMARRT for the algorithm Connected Multi-agent RRT (Rapidly-exploring
   * random tree)
   * */
  template <class GraphMove, class GraphComm>
  class CMARRT : public Solver<GraphMove, GraphComm>
  {
  private:
    int iterations = 0;
    class ExplorationTree
    {
    private:
      float p =
          90;       // bias in % to use the target configuration for the random one
      int step = 1; // TODO: to define
      float neardist = 1;

      std::vector<std::shared_ptr<Configuration>> vertices_;
      // FIXME is parents_[i] the parent node of vertices_[i]?
      std::vector<std::shared_ptr<Configuration>> parents_;
      std::map<Configuration, int> index_of_vertex_;
      std::map<Configuration, int> index_of_parent_;

      const Instance<GraphMove, GraphComm> &instance_;
      const Objective &objective_;

      int index_of_configuration(const Configuration &c)
      {
        if (index_of_vertex_.find(c) == index_of_vertex_.end())
        {
          return -1;
        }
        else
        {
          return index_of_vertex_[c];
        }
        /*
        for (int i = 0; i < vertices_.size(); i++) {
          if (*vertices_[i] == c)
            return i;
        }
        return -1;
        */
      };

      int parent_index_of_configuration(const Configuration &c)
      {
        if (index_of_parent_.find(c) == index_of_parent_.end())
        {
          return -1;
        }
        else
        {
          return index_of_parent_[c];
        }
        /*
        for (int i = 0; i < parents_.size(); i++)
        {
          if (*parents_[i] == c)
            return i;
        }
        return -1;
        */
      };

      /**
       * @brief returns the random configuration (used then to compute the nearest
       * configuration)
       *
       * @param goal
       * @return std::shared_ptr<Configuration>
       */
      std::shared_ptr<Configuration> pick_config_at_random(
          const Configuration &goal)
      {
        int irand = rand() % 100;
        if (irand < this->p)
        {
          // Pick a node of Gc for the 1st agent
          size_t nb_nodes = this->instance().graph().communication().node_count();
          Node first = (uint64_t)rand() % (nb_nodes - 1);
          Configuration configuration;
          configuration.PushBack(first);
          std::unordered_set<Node> neighbors;
          // Pick in the Gc neighbors the position for the 2nd, etc
          for (size_t agt = 1; agt < this->instance().nb_agents(); agt++)
          {
            auto new_neighbors = this->instance().graph().communication().get_neighbors(
                    configuration.at(agt - 1));
            neighbors.insert(new_neighbors.begin(), new_neighbors.end());
                
            auto neighbors_vector =
                std::vector<Node>(neighbors.begin(), neighbors.end());
            auto size = neighbors_vector.size();
            // std::cout << "size for agt " << agt << ":" << size;
            auto nextagt = rand() % (size - 1);
            configuration.PushBack(neighbors_vector.at(nextagt));
            assert(neighbors_vector.at(nextagt) <
                   this->instance().graph().movement().node_count());
          }
          return (std::make_shared<Configuration>(configuration));
        }
        else
        {
          return std::make_shared<Configuration>(goal);
        }
      };

      /**
       * @brief compute the point in the plane that is the barycenter of all
       * locations of agents in config
       *
       * @param config
       * @return std::pair<int, int>
       */
      std::pair<float, float> barycenter(const Configuration &config)
      {
        int nb_agents = config.size();
        assert(nb_agents != 0);
        float x = 0;
        float y = 0;
        for (size_t agt = 0; agt < config.size(); agt++)
        {
          std::pair<int, int> pos_agt =
              this->instance().graph().movement().get_position(config.at(agt));
          x += pos_agt.first;
          y += pos_agt.second;
        }
        return std::make_pair(x / nb_agents, y / nb_agents);
      }

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
      std::shared_ptr<Configuration> get_nearest_configuration(
          const Configuration &rand)
      {
        int mindist = -1;
        std::shared_ptr<Configuration> c_nearest = nullptr;
        std::pair<float, float> rand_pos = barycenter(rand);
        for (auto v : this->get_vertices())
        {
          // distance v->rand
          std::pair<float, float> v_pos = barycenter(*v);
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
      std::shared_ptr<Configuration> get_nearest_configuration_l1(
          const Configuration &c)
      {
        int mindist = -1;
        std::shared_ptr<Configuration> c_nearest = nullptr;

        for (auto v : this->get_vertices())
        {
          int dist = 0;
          for(int agt = 0; agt < this->instance().nb_agents(); agt++){
            auto cpos = this->instance().graph().movement().get_position(c.at(agt));
            auto vpos = this->instance().graph().movement().get_position((*v).at(agt));
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
       * @brief returns configuration from source in the direction of target
       * that configuration is reachable from source
       * if we are lucky, that returned configuration is target
       *
       * @param source
       * @param target
       * @return std::shared_ptr<Configuration>
       */
      std::shared_ptr<Configuration> move_towards(const Configuration &source,
                                                  const Configuration &target)
      {
        std::cout << "Creating small instance\n";  std::cout.flush();
        Instance smallInstance(this->instance().graph(), source, target);
        std::cout << "Creating dfs solver\n";  std::cout.flush();
        coupled::DFS<GraphMove, GraphComm> dfs_solver(smallInstance,
                                                      this->objective_);
        std::cout << "Launching DFS for " << step << " steps\n";  std::cout.flush();
        auto cstart = clock();
        auto cnew =
            std::make_shared<Configuration>(dfs_solver.smallStepCompute(step));
        auto cend = clock();
        assert((*cnew).size() > 0);
        std::cout << "DFS ended in " << (cend-cstart) / CLOCKS_PER_SEC << "s\n";  std::cout.flush();
        return cnew;
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
      std::vector<std::shared_ptr<Configuration>> Neighbors(
          const Configuration &config)
      {
        std::vector<std::shared_ptr<Configuration>> neighbors;
        std::pair<int, int> pos = barycenter(config);
        for (auto c : this->get_vertices())
        {
          std::pair<int, int> c_pos = barycenter(*c);
          int dist =
              (abs(c_pos.first - pos.first) + abs(c_pos.second - pos.second));
          if (dist < this->neardist)
          {
            neighbors.push_back(c);
          }
        }
        return neighbors;
      };

      /**
       * @brief compute the number of steps required to go from first to second
       *
       * @param first
       * @param second
       * @return int
       */
      int Cost(const Configuration &first, const Configuration &second)
      {
        Instance smallInstance(this->instance().graph(), first, second);
        coupled::DFS<GraphMove, GraphComm> dfs_solver(smallInstance,
                                                      this->objective_);
        Execution smallExec = dfs_solver.computeAllPairs();
        return smallExec.size();
      };

      /**
       * @brief replace the parent of a vertex by one providing a shorter path
       *
       * @param Neighborhood
       * @param cmin
       * @param cnew
       */
      void replaceParent( // TODO : test this
          const std::vector<std::shared_ptr<Configuration>> &Neighborhood,
          const Configuration &cmin,
          const Configuration &cnew)
      {
        for (auto c : Neighborhood)
        {
          if (not(cmin == *c) &
              Cost(this->instance_.start(), *c) >
                  Cost(this->instance_.start(), cnew) + Cost(cnew, *c))
          {
            std::cout << "here\n";
            this->parents_.at(index_of_configuration(*c)) =
                std::make_shared<Configuration>(cnew);
            auto parent_index = index_of_configuration(cnew);
            assert(parent_index >= 0);
            this->index_of_parent_[*c] = parent_index;
          }
        }
      };

      /**
       * @brief compute the path between two vertices for the final execution
       *
       * @param first
       * @param second
       * @return smallExec
       */
      std::vector<std::shared_ptr<Configuration>> computeSmallPath(
          const Configuration &first,
          const Configuration &second)
      {
        Instance smallInstance(this->instance().graph(), first, second);
        coupled::DFS<GraphMove, GraphComm> dfs_solver(smallInstance,
                                                      this->objective_);
        Execution exec(dfs_solver.computeAllPairs());
        std::vector<std::shared_ptr<Configuration>> smallExec;
        // std::cout << "\nExecution " << exec;
        for (int i = 1; i < exec.max_path() - 1; i++)
        {
          smallExec.push_back(
              std::make_shared<Configuration>(exec.get_configuration(i)));
        }
        return smallExec;
      }

    public:
      /**
       * @brief Construct a new Exploration Tree that initially contains the
       * starting configuration of the instance.
       *
       *
       *
       * @param instance
       * @param objective
       * @param prob2target probability of picking goal configuration at a given iteration as a direction to expand the tree
       * @param step_size the number of steps the tree is expanded towards a given direction
       */
      explicit ExplorationTree(const Instance<GraphMove, GraphComm> &instance,
                               const Objective &objective,
                               int prob2target,
                               int step_size)
          : instance_(instance), objective_(objective), vertices_(), parents_(), p(prob2target), step(step_size)
      {
        std::cout << "Exploration tree built\n";
        std::shared_ptr<Configuration> start = std::make_shared<Configuration>();
        for (size_t agt = 0; agt < instance.start().size(); agt++)
          start->PushBack(instance.start().at(agt));
        vertices_.push_back(start);
        parents_.push_back(start);
        // step and neardist
        // int size = instance.graph().movement().node_count();
        // step = size / 30;
        // neardist = size / 50;
        std::cout << "Step is " << step << " and neardist " << neardist << "\n";
      };

      std::vector<std::shared_ptr<Configuration>> &get_vertices()
      {
        return vertices_;
      };
      std::vector<std::shared_ptr<Configuration>> &get_parents()
      {
        return parents_;
      };

      void print_vertices()
      {
        for (auto v : this->get_vertices())
        {
          std::cout << *v << "\n";
        }
      }

      void print_vertices_parents()
      {
        std::cout << "There are " << get_parents().size() << " parents, and " << get_vertices().size() << " vertices\n";
        assert(this->get_parents().size() == this->get_vertices().size());
        for (auto v : this->get_vertices())
        {
          assert(this->index_of_configuration(*v) < this->get_parents().size());
          Execution parent_v;
          auto parent = this->get_parents().at(this->index_of_configuration(*v));
          for (size_t agt = 0; agt < this->instance_.nb_agents(); agt++)
          {
            std::shared_ptr<Path> p_agt = std::make_shared<Path>();
            p_agt->PushBack(parent->at(agt));
            p_agt->PushBack(v->at(agt));
            parent_v.set_path(agt, p_agt);
          }
          std::cout << parent_v << "\n";
        }
      }

      const Instance<GraphMove, GraphComm> &instance() { return instance_; };

      bool TreeHasConfig(const Configuration &config)
      {
        return index_of_vertex_.find(config) != index_of_vertex_.end();
        /*
        for (auto found : this->get_vertices())
        {
          if (config == *found)
          {
            return true;
          }
        }
        return false;
        */
      }

      void extend()
      {
        std::cout << "\nTree size: " << vertices_.size() << "\n";
        // this->print_vertices();
        std::shared_ptr<Configuration> c_rand =
            pick_config_at_random(this->instance().goal());
        auto c_rand_barycenter = barycenter(*c_rand);
        if (*c_rand == this->instance().goal()){
          std::cout << ANSI_PURPLE << "Moving towards GOAL: " << ANSI_RESET << (*c_rand) << " barycenter: " << 
            c_rand_barycenter.first << ", " << c_rand_barycenter.second << "\n";
        } else {
          std::cout << ANSI_CYAN << "Moving towards crand: " << (*c_rand) << ANSI_RESET << " barycenter: " <<  
          c_rand_barycenter.first << ", " << c_rand_barycenter.second << "\n";
        }

        std::shared_ptr<Configuration> c_nearest =
            get_nearest_configuration_l1(*c_rand);
        auto c_nearest_barycenter = barycenter(*c_nearest);
        std::cout << "Cnearest is "<< ANSI_YELLOW << *c_nearest << ANSI_RESET << " barycenter: " << 
        c_nearest_barycenter.first << ", " << c_nearest_barycenter.second << "\n";
        std::cout.flush();

        std::shared_ptr<Configuration> c_new = move_towards(*c_nearest, *c_rand);
        auto source = this->instance().start();
        if (c_new->size() == 0)
        {
          std::cout << ANSI_RED << "move_towards failed.\n" << ANSI_RESET;
          std::cout.flush();
          return;
        }
        if (not(this->TreeHasConfig(*c_new)))
        {
          this->vertices_.push_back(c_new);
          this->parents_.push_back(c_nearest);
          this->index_of_vertex_[*c_new] = this->vertices_.size() - 1;
          this->index_of_parent_[*c_new] = this->index_of_vertex_[*c_nearest];
          assert(this->index_of_vertex_.find(*c_nearest) != this->index_of_vertex_.end());
          std::cout << "Adding Cnew: " << ANSI_YELLOW << *c_new << ANSI_RESET << "\n";
          // std::cout << "Cnew added: " << *c_new << "\n";
          // std::cout.flush();
        }
        else
        {
          std::cout << ANSI_RED << "Cnew is already in the tree : " << *c_new << ANSI_RESET << "\n";
        }

        // RRT* component
        //  std::shared_ptr<Configuration> c_min = c_nearest;
        //  int costmin = Cost(source, *c_min) + Cost(*c_min, *c_new);
        //  std::vector<std::shared_ptr<Configuration>> Neighborhood =
        //      Neighbors(*c_new);

        // for (auto cnear : Neighborhood) {
        //   int cost = Cost(source, *cnear) + Cost(*cnear, *c_new);
        //   if (cost < costmin) {
        //     c_min = cnear;
        //     costmin = cost;
        //   }
        // }
        // parents_.push_back(c_min);  // parent of c_new

        // replaceParent(Neighborhood, *c_min, *c_new);
      }

      /**
       * @brief Given an ExplorationTree, compute the execution from start to goal
       *
       * @param start
       * @param goal
       * @return std::vector<std::shared_ptr<Configuration>>
       */
      std::vector<std::shared_ptr<Configuration>> ComputeExecution(
          const Configuration &start,
          const Configuration &goal)
      {
        std::vector<std::shared_ptr<Configuration>> exec;
        std::shared_ptr<Configuration> current =
            std::make_shared<Configuration>(goal);
        exec.insert(exec.begin(), current);
        while (not(*current == start))
        {
          int id = this->index_of_configuration(*current);
          auto parents =
              this->get_parents(); // not necessary anymore except for the assert
          assert(id <= parents.size());
          // FIXME Why id-1? What is this configuration?
          auto parent = this->get_parents().at(id - 1);
          auto smallpath = computeSmallPath(*current, *parent);
          for (int i = 0; i < smallpath.size(); i++)
          {
            exec.insert(exec.begin(), smallpath.at(i));
          }
          exec.insert(exec.begin(), parent);
          current = parent;
        }
        return exec;
      };
      void print_tree(){
        int node_count = this->vertices_.size();
        std::shared_ptr<bool[]> visited(new bool[node_count]);
        for(int i = 0; i < node_count; i++){
          visited[i] = false;
        }
        for(int v = 0; v < node_count; v++){
          if (visited[v]){
            continue;
          }
          while( !visited[v] ){
            visited[v] = true;
            int parent_of_v = this->index_of_parent_[*(this->vertices_[v])];
            if (v != parent_of_v){
              std::cout << "RRT edge: " << (*this->vertices_[v]) << " -- " << (*this->vertices_[parent_of_v]) << "\n";
            }
            v = parent_of_v;
          }
        }
        /*
        std::vector<std::shared_ptr<Configuration>> vertices_;
        // FIXME is parents_[i] the parent node of vertices_[i]?
        std::vector<std::shared_ptr<Configuration>> parents_;
        std::map<Configuration, int> index_of_vertex_;
        std::map<Configuration, int> index_of_parent_;
        */
      }
    };

    ExplorationTree explorationtree_;

  public:
    CMARRT(const Instance<GraphMove, GraphComm> &instance,
           const Objective &objective,
           int prob2target,
           int step_size)
        : Solver<GraphMove, GraphComm>(instance, objective),
          explorationtree_(instance, objective, prob2target, step_size){
            auto cgoal = instance.goal();
            auto cstart = instance.start();
            if (!instance.graph().communication().is_configuration_connected(cstart)){
              LOG_FATAL("Start configuration is not connected.");
              throw "Error";
            }
            if (!instance.graph().communication().is_configuration_connected(cgoal)){
              LOG_FATAL("Goal configuration is not connected.");
              throw "Error";
            }
          };

    bool StepCompute() override
    {
      // std::cout << "\nStep \n";
      if (explorationtree_.TreeHasConfig(this->instance().goal()))
      {
        explorationtree_.print_tree();
        std::cout << "Done after " << iterations << " iterations\n";
        std::cout.flush();
        auto exec = explorationtree_.ComputeExecution(this->instance().start(),
                                                      this->instance().goal());
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
        std::cout << "\n#Iterations: " << iterations << "\n";
        iterations++;
        explorationtree_.extend();
        if (iterations % 10 == 0 ){
          explorationtree_.print_tree();
        }
        return false;
      }
    }
  };

} // namespace cmarrt