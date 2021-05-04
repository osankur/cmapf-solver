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
		virtual void load() = 0;
	};

	class XMLInstanceLoader : public InstanceLoader<ExplicitGraph, ExplicitGraph> {
	private:
		const std::string m_filepath;
		const std::string m_location;

		struct XMLNode
		{
		private:
			const std::string m_id;
			const float m_x;
			const float m_y;

		public:
			XMLNode(const std::string& id, float x, float y) : m_id(id), m_x(x), m_y(y) {};
			const std::string& id() const { return m_id; };
		};

		std::vector<XMLNode> m_nodes;

	public:
		XMLInstanceLoader(const std::string& filepath, const std::string& location);
		void load() override;
	};

	class JSONInstanceLoader : public InstanceLoader<ExplicitGraph, ExplicitGraph> {
	private:
		const std::string m_filepath;
	public:
		JSONInstanceLoader(const std::string& filepath);
		void load() override;
	};
	
}