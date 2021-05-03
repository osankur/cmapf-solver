#pragma once

#include <Path.hpp>
#include <Configuration.hpp>
#include <memory>

class Execution {
private:
	std::vector<std::shared_ptr<const Path>> m_exec;
public:
	void pushBack(std::shared_ptr<const Path>);

	Configuration getConfiguration(uint64_t) const;
	
	void setPath(Agent, std::shared_ptr<const Path>);
	const std::shared_ptr<const Path> getPath(Agent) const;

	// Properties
	size_t size() const;

	// Friends
	friend std::ostream& operator<<(std::ostream&, const Execution&);
};