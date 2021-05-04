#pragma once

#include <Common.hpp>
#include <Graph.hpp>

template<class GraphMove, class GraphComm>
class TopologicalGraph {
private:
	GraphMove m_movementGraph;
	GraphComm m_communicationGraph;
public:
	GraphMove& movement()
	{
		return m_movementGraph;
	}

	GraphComm& communication()
	{
		return m_communicationGraph;
	}

	const GraphMove& movement() const
	{
		return m_movementGraph;
	}

	const GraphComm& communication() const
	{
		return m_communicationGraph;
	}
};