#pragma once

#include <Common.hpp>
#include <Graph.hpp>

template<class GraphMove, class GraphComm>
class TopologicalGraph {
private:
	GraphMove movement_graph_;
	GraphComm communication_graph_;
public:
	GraphMove& movement()
	{
		return movement_graph_;
	}

	GraphComm& communication()
	{
		return communication_graph_;
	}

	const GraphMove& movement() const
	{
		return movement_graph_;
	}

	const GraphComm& communication() const
	{
		return communication_graph_;
	}
};