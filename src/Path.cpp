#include <Path.hpp>
#include <string>

void Path::pushBack(Node node)
{
	m_path.push_back(node);
}

const Node& Path::getAtTimeOrLast(size_t time) const
{
	return time < m_path.size() ? m_path[time] : m_path.back();
}

Node& Path::operator[](size_t time)
{
	return m_path[time];
}

const Node& Path::operator[](size_t time) const
{
	return m_path[time];
}

size_t Path::size() const
{
	return m_path.size();
}

// Friends

std::ostream& operator<<(std::ostream& os, const Path& path)
{
	os << "[";
	for (auto it = path.m_path.cbegin(); it != path.m_path.cend(); ++it)
	{
		os << std::to_string(*it);
		if (std::distance(it, path.m_path.cend()) > 1)
			os << ", ";
	}
	os << "]";
	return os;
}