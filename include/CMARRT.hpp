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

namespace cmarrt {

/**
 * @class CMARRT for the algorithm Connected Multi-agent RRT (Rapidly-exploring
 * random tree)
 * */
template <class GraphMove, class GraphComm>
class CMARRT : public Solver<GraphMove, GraphComm> {
 private:
  class ExplorationTree {
   private:
    float p =
        0;  // bias in % to use the target configuration for the random one
    int step = 1;  // TODO: to define
    float neardist = 1;

    std::vector<std::shared_ptr<Configuration>> vertices_;
    std::vector<std::shared_ptr<Configuration>> parents_;  // for edges

    const Instance<GraphMove, GraphComm>& instance_;
    const Objective& objective_;

    int index_of_configuration(const Configuration& c) {
      for (int i = 0; i < vertices_.size(); i++) {
        if (*vertices_.at(i) == c)
          return i;
      }
      return -1;
    };

    int parent_index_of_configuration(const Configuration& c) {
      for (int i = 0; i < parents_.size(); i++) {
        if (*parents_.at(i) == c)
          return i;
      }
      return -1;
    };

    // int get_index(Configuration& c,
    //               const std::vector<std::shared_ptr<Configuration>>&
    //                   vect) {  // remplacée par les deux méthodes index_of...
    //   auto it = std::find(vect.begin(), vect.end(),
    //                       std::make_shared<Configuration>(c));  // refaire
    //   if (it != vect.end()) {
    //     return it - vect.begin();
    //   }
    //   return -1;
    // };

    /**
     * @brief returns the random configuration (used then to compute the nearest
     * configuration)
     *
     * @param goal
     * @return std::shared_ptr<Configuration>
     */
    std::shared_ptr<Configuration> pick_config_at_random(
        const Configuration& goal) {
      int irand = rand() % 100;
      if (irand < this->p) {
        // Pick a node of Gc for the 1st agent
        size_t nb_nodes = this->instance().graph().communication().node_count();
        Node first = (uint64_t)rand() % (nb_nodes - 1);
        Configuration configuration;
        configuration.PushBack(first);
        // Pick in the Gc neighbors the position for the 2nd, etc
        for (size_t agt = 1; agt < this->instance().nb_agents(); agt++) {
          auto neighbors = this->instance().graph().communication().get_neighbors(
              configuration.at(agt - 1));
          auto next = std::vector<Node>(neighbors.begin(), neighbors.end());
          auto size = next.size();
          std::cout << "size for agt " << agt << ":" << size;
          int random = rand(); 
          std::cout << "rand :" << random;
          std::cout.flush();
          auto nextagt = random % (size-1);
          configuration.PushBack(next.at(nextagt));
          assert(next.at(nextagt) < this->instance().graph().movement().node_count());
        }
        return (std::make_shared<Configuration>(configuration));
      } else {
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
    std::pair<int, int> barycenter(const Configuration& config) {
      int nb_agents = config.size();
      assert(nb_agents != 0);
      int x = 0;
      int y = 0;
      for (size_t agt = 0; agt < config.size(); agt++) {
        std::pair<int, int> pos_agt =
            this->instance().graph().movement().get_position(config.at(agt));
        x += pos_agt.first;
        y += pos_agt.second;
      }
      return std::make_pair((int)x / nb_agents, (int)y / nb_agents);
    }

    /**
     * @brief Get the nearest configuration in the tree in the direction given
     * by the configuration rand. We use the barycenter for evaluating the
     * distance from an existing configuration to rand.
     *
     * @param rand
     * @return std::shared_ptr<Configuration>
     */
    std::shared_ptr<Configuration> get_nearest_configuration(
        const Configuration& rand) {
      int mindist = -1;
      std::shared_ptr<Configuration> c_nearest =
          std::make_shared<Configuration>();
      std::pair<int, int> rand_pos = barycenter(rand);
      for (auto v : this->get_vertices()) {
        // distance v->rand
        std::pair<int, int> v_pos = barycenter(*v);
        int dist = (abs(v_pos.first - rand_pos.first) +
                    abs(v_pos.second - rand_pos.second));
        if (dist < mindist || mindist == -1) {
          mindist = dist;
          c_nearest = v;
        }
      }
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
    std::shared_ptr<Configuration> move_towards(const Configuration& source,
                                                const Configuration& target) {
      Instance smallInstance(this->instance().graph(), source, target);
      coupled::DFS<GraphMove, GraphComm> dfs_solver(smallInstance,
                                                    this->objective_);

      //performs this->step steps of DFS  
      // for (int i = 0; i < this->step; i++) {
        // if (dfs_solver.StepCompute()) {
        //   return 
        //       dfs_solver.get_exec().at(
        //           dfs_solver.get_exec().size());
      //   }
      // }
      //assert(dfs_solver.get_exec().size()>0); //fail
      auto cnew = std::make_shared<Configuration>(dfs_solver.smallStepCompute(step));
      assert((*cnew).size()>0);
      return cnew;
      // return 
      //     dfs_solver.get_exec().at(
      //         dfs_solver.get_exec().size()); //case when the execution does not necessary stop with target


    };

    /**
     * @brief compute the neighbors of the configuration in the tree
     *
     * @param config
     * @return std::vector<std::shared_ptr<Configuration>>
     */
    std::vector<std::shared_ptr<Configuration>> Neighbors(
        const Configuration& config) {
      std::vector<std::shared_ptr<Configuration>> neighbors;
      std::pair<int, int> pos = barycenter(config);
      for (auto c : this->get_vertices()) {
        std::pair<int, int> c_pos = barycenter(*c);
        int dist =
            (abs(c_pos.first - pos.first) + abs(c_pos.second - pos.second));
        if (dist < this->neardist) {
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
    int Cost(const Configuration& first, const Configuration& second) {
      std::vector<std::shared_ptr<Configuration>> smallExec =
          ComputeExecution(*this, first, second);
      return smallExec.size();
    };

    void replaceParent(
        const std::vector<std::shared_ptr<Configuration>>& Neighborhood,
        const Configuration& cmin,
        const Configuration& cnew) {
      for (auto c : Neighborhood) {
        if (not(cmin == *c) &
            Cost(this->instance_.start(), *c) >
                Cost(this->instance_.start(), cnew) + Cost(cnew, *c)) {
          this->parents_.at(index_of_configuration(*c)) =
              std::make_shared<Configuration>(cnew);
        }
      }
    };

   public:
    /**
     * @brief Construct a new Exploration Tree that initially contains the
     * starting configuration of the instance
     *
     * @param instance
     * @param objective
     */
    explicit ExplorationTree(const Instance<GraphMove, GraphComm>& instance,
                             const Objective& objective)
        : instance_(instance), objective_(objective), vertices_(), parents_() {
      std::cout << "Exploration Tree built\n";
      std::shared_ptr<Configuration> start = std::make_shared<Configuration>();
      for (size_t agt = 0; agt < instance.start().size(); agt++)
        start->PushBack(instance.start().at(agt));
      vertices_.push_back(start);
      // step and neardist 
      // int size = instance.graph().movement().node_count();
      // step = size / 10;
      // neardist = size / 50;
    };

    std::vector<std::shared_ptr<Configuration>>& get_vertices() {
      return vertices_;
    };
    std::vector<std::shared_ptr<Configuration>>& get_parents() {
      return parents_;
    };

    void print_vertices() {
      for (auto v : this->get_vertices()){
        std::cout << *v << ". ";
      }
    }

    const Instance<GraphMove, GraphComm>& instance() { return instance_; };

    bool TreeHasConfig(const Configuration& config) {
      for (auto found : this->get_vertices()) {
        if (config == *found) {
          return true;
        }
      }
      return false;
    }

    void extend() {
      std::cout << "Tree has ";
      this->print_vertices();
      std::shared_ptr<Configuration> c_rand =
          pick_config_at_random(this->instance().goal());
      std::cout << "Crand is " << (*c_rand);
      std::shared_ptr<Configuration> c_nearest =
          get_nearest_configuration(*c_rand);
      std::cout << "Cnearest is " << *c_nearest;
      std::shared_ptr<Configuration> c_new = move_towards(*c_nearest, *c_rand);
      auto source = this->instance().start();
      if (c_new->size() == 0)
        return;
      if (not(this->TreeHasConfig(*c_new))) {
        this->vertices_.push_back(c_new);
        this->parents_.push_back(c_nearest);
        std::cout << "Cnew added: "<< *c_new << "\n";
        std::cout.flush();
      }
      // std::shared_ptr<Configuration> c_min = c_nearest;
      // int costmin = Cost(source, *c_min) + Cost(*c_min, *c_new);
      // std::vector<std::shared_ptr<Configuration>> Neighborhood =
      // Neighbors(*c_new);

      // for (auto cnear : Neighborhood){
      //   int cost = Cost(source, *cnear) + Cost(*cnear, *c_new);
      //   if (cost < costmin){
      //     c_min = cnear;
      //     costmin = cost;
      //   }
      // }
      // parents_.push_back(c_min); //parent of c_new

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
        const Configuration& start,
        const Configuration& goal) {
      std::vector<std::shared_ptr<Configuration>> exec;
      std::shared_ptr<Configuration> current =
          std::make_shared<Configuration>(goal);
      exec.insert(exec.begin(), current);
      while (not(*current == start)) {
        int id = this->index_of_configuration(*current);
        auto parents = this->get_parents();
        std::cout.flush();
        assert(id <= parents.size());
        current = this->get_parents().at(id-1);
        exec.insert(exec.begin(), current);
      }
      return exec;
    };
  };

  ExplorationTree explorationtree_;

 public:
  CMARRT(const Instance<GraphMove, GraphComm>& instance,
         const Objective& objective)
      : Solver<GraphMove, GraphComm>(instance, objective),
        explorationtree_(instance, objective){};

  bool StepCompute() override {
    std::cout << "\nStep \n";
    if (explorationtree_.TreeHasConfig(this->instance().goal())) {
      std::cout << "finished\n";
      auto exec = explorationtree_.ComputeExecution(this->instance().start(),
                                                    this->instance().goal());
      for (size_t agt = 0; agt < this->instance_.nb_agents(); agt++) {
        std::shared_ptr<Path> p_agt = std::make_shared<Path>();
        for (size_t t = 0; t < exec.size(); t++) {
          p_agt->PushBack(exec.at(t)->at(agt));
        }
        this->execution_.set_path(agt, p_agt);
      }
      return true;
    }
    explorationtree_.extend();
    return false;
  }
};

}  // namespace cmarrt