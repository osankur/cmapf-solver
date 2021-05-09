#include "doctest.h"

#include <sstream>
#include <Execution.hpp>

TEST_CASE("Testing of Execution class") {
	Execution e;
	CHECK(e.size() == 0);

	Path p1;
	p1.PushBack(1);
	p1.PushBack(2);
	p1.PushBack(3);
	p1.PushBack(4);
	e.PushBack(std::make_shared<const Path>(p1));
	CHECK(e.size() == 1);

	Path p2;
	p2.PushBack(9);
	p2.PushBack(8);
	p2.PushBack(7);
	e.PushBack(std::make_shared<const Path>(p2));

	CHECK(e.size() == 2);
	CHECK(e.get_path(0)->size() == 4);
	CHECK(e.get_path(1)->size() == 3);
	CHECK(e.get_configuration(1).size() == 2);
	CHECK(e.get_configuration(1)[0] == 2);
	CHECK(e.get_configuration(1)[1] == 8);
	CHECK(e.get_configuration(3).size() == 2);
	CHECK(e.get_configuration(3)[0] == 4);
	CHECK(e.get_configuration(3)[1] == 7);

	Path p3;
	p3.PushBack(15);
	p3.PushBack(16);
	p3.PushBack(17);
	e.set_path(0, std::make_shared<const Path>(p3));

	CHECK(e.size() == 2);
	CHECK(e.get_path(0)->size() == 3);
	CHECK(e.get_path(1)->size() == 3);
	CHECK(e.get_configuration(3).size() == 2);
	CHECK(e.get_configuration(3)[0] == 17);
	CHECK(e.get_configuration(3)[1] == 7);

	std::stringstream out;
	out << e;
	CHECK(out.str() == "{[15, 16, 17], [9, 8, 7]}");
}
