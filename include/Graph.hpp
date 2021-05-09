#pragma once

#include <Common.hpp>
#include <vector>
#include <set>

class Graph {
public:
	virtual const std::set<Node>& get_neighbors(Node) const = 0;
};

class ExplicitGraph : public Graph {
private:
	std::vector<std::set<Node>> adjacency_;
public:
	void AddNode(Node);
	void AddEdge(Node, Node);

	size_t node_count() const;
	size_t edge_count() const;
	const std::set<Node>& get_neighbors(Node) const override;
};

class RadiusGraph : public Graph {
public:
	const std::set<Node>& get_neighbors(Node) const override;
};

class GridGraph : public Graph {
public:
	const std::set<Node>& get_neighbors(Node) const override;
};