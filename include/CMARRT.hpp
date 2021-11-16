#pragma once

#include <CTNOrderingStrategy.hpp>
#include <ConflictSelectionStrategy.hpp>
#include <ConstraintTreeNode.hpp>
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
 float p = 20; //biasis in %
 int step = 5; //to define
  class ExplorationTree {
   public:
    std::vector<std::shared_ptr<Configuration>> vertices_;
	  std::vector<std::shared_ptr<Configuration>> parents_; //for edges

    const Instance<GraphMove, GraphComm>& instance_;

    explicit ExplorationTree(const Instance<GraphMove, GraphComm>& instance) 
    : instance_(instance), vertices_(), parents_() {
      std::shared_ptr<Configuration> start = std::make_shared<Configuration>();
      for (size_t agt = 0; agt < instance.start().size(); agt++) start->PushBack(instance.start()[agt]);
      vertices_.push_back(start);
    };

    std::vector<std::shared_ptr<Configuration>> get_vertices() { return *vertices_;};
    std::vector<std::shared_ptr<Configuration>> get_parents() { return parents_;};

    int get_index(Configuration c, const std::vector<std::shared_ptr<Configuration>> vect){
      auto it = std::find(vect.begin(), vect.end(), std::make_shared<Configuration>(c));
      if (it != vect.end()){
        return it - vect.begin();
      }
      return -1;
    };

    bool TreeHasConfig(const std::shared_ptr<Configuration>& config) {
      for (auto found : this->explorationtree_) {
        if (config->size() == found->size()) {
          bool foundisconfig = true;
          for (size_t agt = 0; agt < config->size(); agt++) {
            if (config->at(agt) != found->at(agt)) {
              foundisconfig = false;
            }
          }
          if (foundisconfig)
            return true;
        }
      }
    }

    void extend(tree){
      std::shared_ptr<Configuration> c_rand = pick_rand(p, std::make_shared(tree->goal()));
      std::shared_ptr<Configuration> c_nearest = nearest(c_rand);
      std::shared_ptr<Configuration> c_new = few_steps(c_rand, c_nearest);

      vertices_.push_back(c_new);
      std::shared_ptr<Configuration> c_min = c_nearest;
      int costmin = Cost(sources, c_min) + Cost(c_min, c_new);
      std::vector<std::shared_ptr<Configuration>> Neighborhood = Neighbors(c_new);

      for (auto cnear : Neighborhood){
        int cost = Cost(sources, cnear) + Cost(cnear, cnew);
        if (cost < costmin){
          c_min = cnear;
          costmin = cost;
        }
      }
      parents_.push_back(c_min); //parent of c_new

      replaceParent(Neighborhood, c_min, c_new);
      
    }

    std::shared_ptr<Configuration> pick_rand(float prob, const std::shared_ptr<Configuration> goal){
      srand(time(NULL));
      int irand = rand()%100;
      if (irand < prob){
        //TODO
      } else {
        return goal;
      }
    };

    std::shared_ptr<Configuration> nearest(const std::shared_ptr<Configuration>& rand){
      float mindist = -1;
      std::shared_ptr<Configuration> c_nearest = std::make_shared<Configuration>();
      for (auto v: this->get_vertices()){
        //distance v->rand TODO
      }
      return c_nearest;
    };

    std::shared_ptr<Configuration> few_steps(const std::shared_ptr<Configuration>& rand, const std::shared_ptr<Configuration>& nearest){
      //DFS source = nearest & target = rand
    };

    std::vector<std::shared_ptr<Configuration>> Neighbors(const std::shared_ptr<Configuration>& config){
      //
    };

    int Cost(const std::shared_ptr<Configuration>& first, const std::shared_ptr<Configuration>& second){};

    void replaceParent(const std::vector<std::shared_ptr<Configuration>>& Neighborhood, const std::shared_ptr<Configuration>& min, const std::shared_ptr<Configuration>& near){};
  }

  ExplorationTree<GraphMove, GraphComm> explorationtree_;

  std::vector<std::shared_ptr<Configuration>> ComputePath(ExplorationTree tree, Instance instance)
  {
    std::vector<std::shared_ptr<Configuration>> exec;
  }

 public:
  CMARRT(const Instance<GraphMove, GraphComm>& instance, const Objective& objective)
      : Solver<GraphMove, GraphComm>(instance, objective), explorationtree_(instance){};

  bool StepCompute() override {
    if (explorationtree_.TreeHasConfig(this->instance_->goal())) {
      this->execution_ = ComputePath(this->explorationtree_, this->instance_);
      return true;
    }
    extend(this->explorationtree_);
    return false;
  }
};

}  // namespace cmarrt