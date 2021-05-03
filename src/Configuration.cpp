#include <Configuration.hpp>
#include <string>

void Configuration::pushBack(Node node)
{
	m_config.push_back(node);
}

Node& Configuration::operator[](Agent a)
{
	return m_config[a];
}

const Node& Configuration::operator[](Agent a) const
{
	return m_config[a];
}

size_t Configuration::size() const
{
	return m_config.size();
}

// Friends

std::ostream& operator<<(std::ostream& os, const Configuration& config)
{
	os << "<";
	for (auto it = config.m_config.cbegin(); it != config.m_config.cend(); ++it)
	{
		os << std::to_string(*it);
		if (std::distance(it, config.m_config.cend()) > 1)
			os << ", ";
	}
	os << ">";
	return os;
}