#pragma once

#include <TopologicalGraph.hpp>
#include <Configuration.hpp>

namespace instance {
	template<class GraphMove, class GraphComm>
	class Instance {
	private:
		TopologicalGraph<GraphMove, GraphComm> m_topoGraph;
		Configuration m_start;
		Configuration m_goal;
		size_t m_nbAgents;
	public:
		
		TopologicalGraph<GraphMove, GraphComm>& graph() { return m_topoGraph; }
		Configuration& start() { return m_start; }
		Configuration& goal() { return m_goal; }
		void setNbAgents(size_t nb) { m_nbAgents = nb; }

		const TopologicalGraph<GraphMove, GraphComm>& graph() const { return m_topoGraph; }
		const Configuration& start() const { return m_start; }
		const Configuration& goal() const { return m_goal; }
		size_t nbAgents() const { return m_nbAgents; }
	};

}