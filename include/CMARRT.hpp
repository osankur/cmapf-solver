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

template <class GraphMove, class GraphComm>
class CMARRT : public Solver<GraphMove, GraphComm> {
 private:
  class ExplorationTree {
   private:
    float p = 2;  // biasis in %
    int step = 50;  // to define
    float neardist = 10;

    std::vector<std::shared_ptr<Configuration>> vertices_;
    std::vector<std::shared_ptr<Configuration>> parents_;  // for edges

    const Instance<GraphMove, GraphComm>& instance_;
    const Objective& objective_;

    int index_of_configuration(const Configuration& c) {
      for (int i = 0; i < vertices_.size(); i++) {
        if (*vertices_[i] == c)
          return i;
      }
      return -1;
    };

    int parent_index_of_configuration(const Configuration& c) {
      for (int i = 0; i < parents_.size(); i++) {
        if (*parents_[i] == c)
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

    std::shared_ptr<Configuration> pick_state_at_random(
        const Configuration& goal) {
      int irand = rand() % 100;
      if (irand < this->p) {
        // TODO
        // Tirer (entier) noeud de Gc au hasard pour 1er agent
        size_t nb_nodes = this->instance().graph().communication().node_count();
        Node first = (uint64_t)rand() % nb_nodes - 1;
        Configuration agents;
        agents.PushBack(first);
        // Tirer dans les voisins (dans Gc) la position du 2e, etc...
        for (size_t agt = 1; agt < this->instance().nb_agents(); agt++) {
          auto next = this->instance().graph().communication().get_neighbors(
              agents[agt - 1]);
          int nextagt = rand() % next.size();
          auto it = next.begin();
          for (int i = 0; i < nextagt; i++) {
            it++;
          }
          agents.PushBack(*it);
        }
        return (std::make_shared<Configuration>(agents));
      } else {
        return std::make_shared<Configuration>(goal);
      }
    };

    std::pair<int, int> barycenter(const Configuration& config) {
      int nb_agents = config.size();
      int x = 0;
      int y = 0;
      for (size_t agt = 0; agt < config.size(); agt++) {
        std::pair<int, int> pos_agt =
            this->instance().graph().movement().get_position(config[agt]);
        x += pos_agt.first;
        y += pos_agt.second;
      }
      return std::make_pair((int)x / nb_agents, (int)y / nb_agents);
    }

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

    std::shared_ptr<Configuration> move_towards(const Configuration& rand,
                                                const Configuration& nearest) {
      // DFS source = nearest & target = rand
      // Instance smallInstance(this->instance().graph(), rand, nearest); //copy

      Instance smallInstance(this->instance().graph(), rand, nearest);
      coupled::DFS<GraphMove, GraphComm> dfs_solver(smallInstance,
                                                    this->objective_);
      for (int i = 0; i < this->step; i++) {
        if (dfs_solver.smallStepCompute()) {
          return std::make_shared<Configuration>(
              dfs_solver.execution().get_configuration(
                  dfs_solver.execution().size()));
        }
      }
      return std::make_shared<Configuration>(
          dfs_solver.execution().get_configuration(
              dfs_solver.execution().size()));
    };

    std::vector<std::shared_ptr<Configuration>> Neighbors(const Configuration& config){
        std::vector<std::shared_ptr<Configuration>> neighbors;
        std::pair<int, int> pos = barycenter(config);
        for (auto c: this->get_vertices()){
          std::pair<int, int> c_pos = barycenter(*c);
          int dist = (abs(c_pos.first - pos.first) +
                    abs(c_pos.second - pos.second));
          if (dist < this->neardist) {
            neighbors.push_back(c);
        }
        }
        return neighbors;
    };

    int Cost(const Configuration& first, const Configuration& second) {
      std::vector<std::shared_ptr<Configuration>> smallExec =
          ComputePath(*this, first, second);
      return smallExec.size();
    };

    void replaceParent(const std::vector<std::shared_ptr<Configuration>>& Neighborhood, const Configuration& cmin, const Configuration& cnew) {
      for (auto c : Neighborhood) {
        if (not(cmin == *c) &
            Cost(this->instance_.start(), *c) >
                Cost(this->instance_.start(), cnew) + Cost(cnew, *c)) {
          this->parents_[index_of_configuration(*c)] =
              std::make_shared<Configuration>(cnew);
        }
      }
    };

   public:
    explicit ExplorationTree(const Instance<GraphMove, GraphComm>& instance,
                             const Objective& objective)
        : instance_(instance), objective_(objective), vertices_(), parents_() {
      std::shared_ptr<Configuration> start = std::make_shared<Configuration>();
      for (size_t agt = 0; agt < instance.start().size(); agt++)
        start->PushBack(instance.start()[agt]);
      vertices_.push_back(start);
    };

    std::vector<std::shared_ptr<Configuration>>& get_vertices() {
      return vertices_;
    };
    std::vector<std::shared_ptr<Configuration>>& get_parents() {
      return parents_;
    };
    const Instance<GraphMove, GraphComm>& instance() { return instance_; };

    bool TreeHasConfig(const Configuration& config) {
      for (auto found : this->get_vertices()) {
        if (config == *found) {
          return true;
        }
        return false;
      }
    }

    void extend() {
      std::shared_ptr<Configuration> c_rand =
          pick_state_at_random(this->instance().goal());
      std::shared_ptr<Configuration> c_nearest =
          get_nearest_configuration(*c_rand);
      std::shared_ptr<Configuration> c_new = move_towards(*c_rand, *c_nearest);
      auto source = this->instance().start();

      this->vertices_.push_back(c_new);
      this->parents_.push_back(c_nearest);

      std::cout << c_new;
      std::cout.flush();
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

    std::vector<std::shared_ptr<Configuration>> ComputePath(
        const Configuration& start,
        const Configuration& goal) {
      std::vector<std::shared_ptr<Configuration>> exec;
      std::shared_ptr<Configuration> current =
          std::make_shared<Configuration>(goal);
      exec.insert(exec.begin(), current);
      while (not(*current == start)) {
        int id = this->index_of_configuration(*current);
        current = this->get_parents()[id];
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
    if (explorationtree_.TreeHasConfig(this->instance().goal())) {
      auto exec = explorationtree_.ComputePath(this->instance().start(),
                                               this->instance().goal());
      for (size_t agt = 0; agt < this->instance_.nb_agents(); agt++) {
        std::shared_ptr<Path> p_agt = std::make_shared<Path>();
        for (size_t t = 0; t < exec.size(); t++) {
          p_agt->PushBack(exec[t]->at(agt));
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