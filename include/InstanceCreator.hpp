#pragma once

#include <Instance.hpp>

namespace instance {

	template<class GraphMove, class GraphComm>
	class InstanceCreator {
	protected:
		Instance<GraphMove, GraphComm> m_instance;
	public:
		const Instance<GraphMove, GraphComm>& instance() const { return m_instance; };
	};

	template<class GraphMove, class GraphComm>
	class InteractiveInstanceCreator : public InstanceCreator<GraphMove, GraphComm> {

	};

}