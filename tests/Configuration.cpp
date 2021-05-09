#include "doctest.h"

#include <sstream>
#include <Configuration.hpp>

TEST_CASE("Testing of Configuration class") {
	Configuration c;
	CHECK(c.size() == 0);
	c.PushBack(1);
	CHECK(c.size() == 1);
	c.PushBack(2);
	CHECK(c.size() == 2);
	CHECK(c[0] == 1);
	CHECK(c[1] == 2);
	c[0] = 3;
	CHECK(c[0] == 3);

	const Configuration c2(c);
	CHECK(c2.size() == 2);
	CHECK(c2[1] == 2);
	CHECK(c2[0] == 3);

	std::stringstream out;
	out << c;
	CHECK(out.str() == "<3, 2>");
	c.PushBack(6);
	out.str(std::string());
	out.clear();
	out << c;
	CHECK(out.str() == "<3, 2, 6>");
}
