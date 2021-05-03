#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

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
}
