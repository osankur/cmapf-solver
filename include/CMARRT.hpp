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
  class Exploration {
   public:
    // explorationtree_; structure ?
    const Instance<GraphMove, GraphComm>& instance_;

    bool has(const std::shared_ptr<Configuration>& config) {
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
  }

  std::vector<std::shared_ptr<Configuration>> vertices_;
	std::vector<std::shared_ptr<Configuration>> parents_; //for edges

 public:
  CMARRT(const Instance<GraphMove, GraphComm>& instance, const Objective& objective)
      : Solver<GraphMove, GraphComm>(instance, objective), exec_() {
        	std::shared_ptr<Configuration> start = std::make_shared<Configuration>();
          for (size_t agt = 0; agt < instance.start().size(); agt++) start->PushBack(instance;start()[agt]);
					vertices_.push_back(start);
			}

  bool StepCompute() override {
    if (explorationtree_.has(instance_->goal())) {
      this->execution_ = ComputePath(explorationtree_, instance_) return true
    }
    extend(explorationtree_);
    return false
  }
};

}  // namespace cmarrt