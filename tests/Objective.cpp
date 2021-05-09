#include "doctest.h"

#include <Objective.hpp>

TEST_CASE("Testing of MaxObjective class") {
	Execution e;

	MaxObjective max;

	CHECK(max.cost(e) == 0);

	Path p1;
	p1.PushBack(1);
	p1.PushBack(2);
	p1.PushBack(3);
	p1.PushBack(4);
	e.PushBack(std::make_shared<const Path>(p1));

	CHECK(max.cost(e) == 4);

	Path p2;
	p2.PushBack(9);
	p2.PushBack(8);
	p2.PushBack(7);
	e.PushBack(std::make_shared<const Path>(p2));

	CHECK(max.cost(e) == 4);

	Path p3;
	p3.PushBack(15);
	p3.PushBack(16);
	p3.PushBack(17);
	p3.PushBack(18);
	p3.PushBack(19);
	e.set_path(0, std::make_shared<const Path>(p3));

	CHECK(max.cost(e) == 5);

	Path p4;
	p4.PushBack(15);
	p4.PushBack(16);
	p4.PushBack(17);
	p4.PushBack(18);
	p4.PushBack(19);
	p4.PushBack(20);
	e.PushBack(std::make_shared<const Path>(p4));

	CHECK(max.cost(e) == 6);
}

TEST_CASE("Testing of SumObjective class") {
	Execution e;

	SumObjective sum;

	CHECK(sum.cost(e) == 0);

	Path p1;
	p1.PushBack(1);
	p1.PushBack(2);
	p1.PushBack(3);
	p1.PushBack(4);
	e.PushBack(std::make_shared<const Path>(p1));

	CHECK(sum.cost(e) == 4);

	Path p2;
	p2.PushBack(9);
	p2.PushBack(8);
	p2.PushBack(7);
	e.PushBack(std::make_shared<const Path>(p2));

	CHECK(sum.cost(e) == 7);

	Path p3;
	p3.PushBack(15);
	p3.PushBack(16);
	p3.PushBack(17);
	p3.PushBack(18);
	p3.PushBack(19);
	e.set_path(0, std::make_shared<const Path>(p3));

	CHECK(sum.cost(e) == 8);

	Path p4;
	p4.PushBack(15);
	p4.PushBack(16);
	p4.PushBack(17);
	p4.PushBack(18);
	p4.PushBack(19);
	p4.PushBack(20);
	e.PushBack(std::make_shared<const Path>(p4));

	CHECK(sum.cost(e) == 14);
}