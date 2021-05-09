#pragma once

#include <Instance.hpp>
#include <Objective.hpp>
#include <Execution.hpp>

class Solver {
private:
	const instance::Instance& instance_;
	const Objective& objective_;
	Execution execution_;
public:
	Solver(const Instance& instance, const Objective& objective) : instance_(instance), objective_(objective), execution_() {}

	virtual bool StepCompute() = 0;

	const Execution Compute() {
		while (!StepCompute());
		return execution_;
	}

	const instance::Instance& instance() const { return instance_; }
	const Objective& objective() const { return objective_; }
};