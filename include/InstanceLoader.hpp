#pragma once

#include <sstream>

#include <InstanceCreator.hpp>

#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"

namespace instance {
	template<class GraphMove, class GraphComm>
	class InstanceLoader : public InstanceCreator<GraphMove, GraphComm> {
	public:
		InstanceLoader(){};
		virtual void Load() = 0;
	};

	class XMLInstanceLoader : public InstanceLoader<ExplicitGraph, ExplicitGraph> {
	private:
		const std::string file_path_;
		const std::string graph_folder_;

		struct XMLNode
		{
		private:
			const std::string id_;
			const float x_;
			const float y_;

		public:
			XMLNode(const std::string& id, float x, float y) : id_(id), x_(x), y_(y) {};
			const std::string& id() const { return id_; };
		};

		std::vector<XMLNode> nodes_;

	public:
		XMLInstanceLoader(const std::string& filepath, const std::string& location);
		void Load() override;
	};

	class JSONInstanceLoader : public InstanceLoader<ExplicitGraph, ExplicitGraph> {
	private:
		const std::string file_path_;
	public:
		explicit JSONInstanceLoader(const std::string& filepath);
		void Load() override;
	};
	
}