#pragma once

#include <Instance.hpp>

namespace instance {

	template<class GraphMove, class GraphComm>
	class InstanceCreator {
	protected:
		Instance<GraphMove, GraphComm> instance_;
	public:
		const Instance<GraphMove, GraphComm>& instance() const { return instance_; };
	};

	template<class GraphMove, class GraphComm>
	class InteractiveInstanceCreator : public InstanceCreator<GraphMove, GraphComm> {

	};

}