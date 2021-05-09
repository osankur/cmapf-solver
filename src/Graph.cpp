#include <Graph.hpp>

size_t ExplicitGraph::node_count() const {
	return adjacency_.size();
}

size_t ExplicitGraph::edge_count() const {
	size_t sum = 0;
	for (auto& edges : adjacency_)
		sum += edges.size();
	return sum;
}

void ExplicitGraph::AddNode(Node n)
{
	if (adjacency_.size() <= n)
		adjacency_.resize(n);
	adjacency_.insert(adjacency_.begin() + n, std::set<Node>());
}

void ExplicitGraph::AddEdge(Node source, Node target)
{
	adjacency_[source].insert(target);
}

const std::set<Node>& ExplicitGraph::get_neighbors(Node n) const
{
	return adjacency_[n];
}