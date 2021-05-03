#pragma once
#include <Common.hpp>

#include <iostream>
#include <vector>

class Path {
private:
	std::vector<Node> m_path;

public:
	/*
	Path();											// Constructor
	~Path();										// Deconstructor
	Path(const Path& other);						// Copy constructor
	Path(Path&& other);								// Move constructor
	Path& operator=(const Path& other) noexcept;	// Copy assignement
	Path& operator=(Path&& other) noexcept;			// Move assignement
	*/

	void pushBack(Node);
	const Node& getAtTimeOrLast(size_t) const;
	
	// Operators
	Node& operator[](size_t);
	const Node& operator[](size_t) const;

	// Properties
	size_t size() const;

	// Friends
	friend std::ostream& operator<<(std::ostream& os, const Path&);
};
