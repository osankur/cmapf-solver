#pragma once

#include <vector>

class Path {
private:
	std::vector<uint64_t> m_path;

public:
	/*
	Path();								// Constructor
	~Path();							// Deconstructor
	Path(const Path& other);			// Copy constructor
	Path(Path&& other);					// Move constructor
	Path& operator=(const Path& other); // Copy assignement
	Path& operator=(Path&& other);		// Move assignement
	*/

	void pushBack(uint64_t);
	
	uint64_t& operator[](uint64_t);
	const uint64_t& operator[](uint64_t) const;
	uint64_t size() const;
};