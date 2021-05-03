#include <Execution.hpp>

void Execution::pushBack(std::shared_ptr<const Path> p)
{
	m_exec.push_back(p);
}

Configuration Execution::getConfiguration(uint64_t time) const
{
	Configuration c;
	for (Agent a = 0; a < static_cast<Agent>(m_exec.size()); a++)
	{
		c.pushBack(m_exec[a]->getAtTimeOrLast(time));
	}
	return c;
}

void Execution::setPath(Agent a, std::shared_ptr<const Path> p)
{
	m_exec[a] = p;
}

const std::shared_ptr<const Path> Execution::getPath(Agent a) const
{
	return m_exec[a];
}

const size_t Execution::size() const
{
	return m_exec.size();
}

std::ostream& operator<<(std::ostream& os, const Execution& e)
{
	os << "[";
	for (auto it = e.m_exec.cbegin(); it != e.m_exec.cend(); ++it)
	{
		os << **it;
		if (std::distance(it, e.m_exec.cend()) > 1)
			os << ", ";
	}
	os << "]";
	return os;
}
