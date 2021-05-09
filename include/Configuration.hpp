#pragma once

#include <Common.hpp>
#include <vector>
#include <ostream>

class Configuration {
private:
	std::vector<Node> config_;

public:
	void PushBack(Node);

	// Operators
	Node& operator[](Agent);
	const Node& operator[](Agent) const;

	// Properties
	size_t size() const;

	// Friends
	friend std::ostream& operator<<(std::ostream&, const Configuration&);
};