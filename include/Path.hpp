#pragma once
#include <Common.hpp>

#include <iostream>
#include <vector>

class Path {
private:
	std::vector<Node> path_;

public:
	void PushBack(Node);
	const Node& GetAtTimeOrLast(size_t) const;
	
	// Operators
	Node& operator[](size_t);
	const Node& operator[](size_t) const;

	// Properties
	size_t size() const;

	// Friends
	friend std::ostream& operator<<(std::ostream&, const Path&);
};
