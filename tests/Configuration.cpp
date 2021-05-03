#include "doctest.h"

#include <Configuration.hpp>

TEST_CASE("Testing of Configuration class") {
	Configuration c;
	CHECK(c.size() == 0);
	c.pushBack(1);
	CHECK(c.size() == 1);
	c.pushBack(2);
	CHECK(c.size() == 2);
	CHECK(c[0] == 1);
	CHECK(c[1] == 2);
	c[0] = 3;
	CHECK(c[0] == 3);
}
