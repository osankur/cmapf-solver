#pragma once

#include <Path.hpp>
#include <Configuration.hpp>
#include <memory>

class Execution {
private:
	std::vector<std::shared_ptr<const Path>> exec_;
public:
	void PushBack(std::shared_ptr<const Path>);

	Configuration get_configuration(uint64_t) const;
	
	void set_path(Agent, std::shared_ptr<const Path>);
	const std::shared_ptr<const Path> get_path(Agent) const;

	// Properties
	size_t size() const;

	// Friends
	friend std::ostream& operator<<(std::ostream&, const Execution&);
};