#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <sstream>
#include <Path.hpp>

TEST_CASE("Testing of Path class") {
	Path p;
	CHECK(p.size() == 0);
	p.pushBack(1);
	CHECK(p.size() == 1);
	p.pushBack(2);
	CHECK(p.size() == 2);
	CHECK(p[0] == 1);
	CHECK(p[1] == 2);
	p[0] = 3;
	CHECK(p[0] == 3);

	const Path p2(p);
	CHECK(p2.size() == 2);
	CHECK(p2[1] == 2);
	CHECK(p2[0] == 3);

	std::stringstream out;
	out << p;
	CHECK(out.str() == "[3, 2]");
	p.pushBack(6);
	out.str(std::string());
	out.clear();
	out << p;
	CHECK(out.str() == "[3, 2, 6]");
}
