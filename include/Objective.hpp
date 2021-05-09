#pragma once

#include <Execution.hpp>

class Objective {
public:
	virtual size_t cost(const Execution&) const = 0;
};

class MaxObjective : public Objective {
public:
	size_t cost(const Execution& e) const override {
		size_t max = 0;
		for (Agent i = 0; i < static_cast<Agent>(e.size()); i++) {
			size_t size = e.get_path(i)->size();
			if (max < size)
				max = size;
		}
		return max;
	}
};

class SumObjective : public Objective {
public:
	size_t cost(const Execution& e) const override {
		size_t sum = 0;
		for (Agent i = 0; i < static_cast<Agent>(e.size()); i++)
			sum += e.get_path(i)->size();
		return sum;
	}
};