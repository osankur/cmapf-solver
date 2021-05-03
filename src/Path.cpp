#include <Path.hpp>

void Path::pushBack(uint64_t node)
{
	m_path.push_back(node);
}

uint64_t& Path::operator[](uint64_t time)
{
	return m_path[time];
}

const uint64_t& Path::operator[](uint64_t time) const
{
	return m_path[time];
}

uint64_t Path::size() const
{
	return m_path.size();
}
