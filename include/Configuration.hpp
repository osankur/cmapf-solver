#pragma once

#include <Common.hpp>
#include <vector>
#include <ostream>

class Configuration {
private:
	std::vector<Node> m_config;
public:
	/*
	Configuration();													// Constructor
	~Configuration();													// Deconstructor
	Configuration(const Configuration& other);							// Copy constructor
	Configuration(Configuration&& other);								// Move constructor
	Configuration& operator=(const Configuration& other) noexcept;		// Copy assignement
	Configuration& operator=(Configuration&& other) noexcept;			// Move assignement
	*/

	void pushBack(Node);

	// Operators
	Node& operator[](Agent);
	const Node& operator[](Agent) const;

	// Properties
	size_t size() const;

	// Friends
	friend std::ostream& operator<<(std::ostream&, const Configuration&);
};