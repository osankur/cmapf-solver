#include "doctest.h"

#include <sstream>
#include <Execution.hpp>

TEST_CASE("Testing of Execution class") {
	Execution e;
	CHECK(e.size() == 0);

	Path p1;
	p1.pushBack(1);
	p1.pushBack(2);
	p1.pushBack(3);
	p1.pushBack(4);
	e.pushBack(std::make_shared<const Path>(p1));
	CHECK(e.size() == 1);

	Path p2;
	p2.pushBack(9);
	p2.pushBack(8);
	p2.pushBack(7);
	e.pushBack(std::make_shared<const Path>(p2));

	CHECK(e.size() == 2);
	CHECK(e.getPath(0)->size() == 4);
	CHECK(e.getPath(1)->size() == 3);
	CHECK(e.getConfiguration(1).size() == 2);
	CHECK(e.getConfiguration(1)[0] == 2);
	CHECK(e.getConfiguration(1)[1] == 8);
	CHECK(e.getConfiguration(3).size() == 2);
	CHECK(e.getConfiguration(3)[0] == 4);
	CHECK(e.getConfiguration(3)[1] == 7);

	Path p3;
	p3.pushBack(15);
	p3.pushBack(16);
	p3.pushBack(17);
	e.setPath(0, std::make_shared<const Path>(p3));

	CHECK(e.size() == 2);
	CHECK(e.getPath(0)->size() == 3);
	CHECK(e.getPath(1)->size() == 3);
	CHECK(e.getConfiguration(3).size() == 2);
	CHECK(e.getConfiguration(3)[0] == 17);
	CHECK(e.getConfiguration(3)[1] == 7);

	std::stringstream out;
	out << e;
	CHECK(out.str() == "{[15, 16, 17], [9, 8, 7]}");
}
