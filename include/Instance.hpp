#pragma once

#include <TopologicalGraph.hpp>
#include <Configuration.hpp>

namespace instance {

	template<class GraphMove, class GraphComm>
	class Instance {
	private:
		TopologicalGraph<GraphMove, GraphComm> topo_graph_;
		Configuration start_;
		Configuration goal_;
		size_t nb_agents_;

	public:
		Instance() : topo_graph_(), start_(), goal_(), nb_agents_(0) {}
		~Instance()									= default;
		Instance(const Instance& other)				= default;
		Instance(Instance&& other)					= default;
		Instance& operator=(const Instance& other)	= default;
		Instance& operator=(Instance&& other)		= default;

		TopologicalGraph<GraphMove, GraphComm>& graph() { return topo_graph_; }
		Configuration& start() { return start_; }
		Configuration& goal() { return goal_; }
		void set_nb_agents(size_t nb) { nb_agents_ = nb; }

		const TopologicalGraph<GraphMove, GraphComm>& graph() const { return topo_graph_; }
		const Configuration& start() const { return start_; }
		const Configuration& goal() const { return goal_; }
		size_t nb_agents() const { return nb_agents_; }
	};

}