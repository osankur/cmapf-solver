#pragma once

#include <Common.hpp>
#include <vector>
#include <set>

class Graph {
public:
	virtual const std::set<Node>& getNeighbors(Node) const = 0;
};

class ExplicitGraph : public Graph {
private:
	std::vector<std::set<Node>> m_adjacency;
public:
	void addNode(Node);
	void addEdge(Node, Node);

	size_t getNodeCount() const;
	size_t getEdgeCount() const;
	const std::set<Node>& getNeighbors(Node) const override;
};

class RadiusGraph : public Graph {
public:
	const std::set<Node>& getNeighbors(Node) const override;
};

class GridGraph : public Graph {
public:
	const std::set<Node>& getNeighbors(Node) const override;
};