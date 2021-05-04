#include <Graph.hpp>

size_t ExplicitGraph::getNodeCount() const {
	return m_adjacency.size();
}

size_t ExplicitGraph::getEdgeCount() const {
	size_t sum = 0;
	for (auto& edges : m_adjacency)
		sum += edges.size();
	return sum;
}

void ExplicitGraph::addNode(Node n)
{
	if (m_adjacency.size() <= n)
		m_adjacency.resize(n);
	m_adjacency.insert(m_adjacency.begin() + n, std::set<Node>());
}

void ExplicitGraph::addEdge(Node source, Node target)
{
	m_adjacency[source].insert(target);
}

const std::set<Node>& ExplicitGraph::getNeighbors(Node n) const
{
	return m_adjacency[n];
}