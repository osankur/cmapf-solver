#include <Configuration.hpp>
#include <string>

void Configuration::PushBack(Node node)
{
	config_.push_back(node);
}

Node& Configuration::operator[](Agent a)
{
	return config_[a];
}

const Node& Configuration::operator[](Agent a) const
{
	return config_[a];
}

size_t Configuration::size() const
{
	return config_.size();
}

// Friends

std::ostream& operator<<(std::ostream& os, const Configuration& config)
{
	os << "<";
	for (auto it = config.config_.cbegin(); it != config.config_.cend(); ++it)
	{
		os << std::to_string(*it);
		if (std::distance(it, config.config_.cend()) > 1)
			os << ", ";
	}
	os << ">";
	return os;
}