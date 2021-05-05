#include "doctest.h"

#include <Objective.hpp>

TEST_CASE("Testing of MaxObjective class") {
	Execution e;

	MaxObjective max;

	CHECK(max.getCost(e) == 0);

	Path p1;
	p1.pushBack(1);
	p1.pushBack(2);
	p1.pushBack(3);
	p1.pushBack(4);
	e.pushBack(std::make_shared<const Path>(p1));

	CHECK(max.getCost(e) == 4);

	Path p2;
	p2.pushBack(9);
	p2.pushBack(8);
	p2.pushBack(7);
	e.pushBack(std::make_shared<const Path>(p2));

	CHECK(max.getCost(e) == 4);

	Path p3;
	p3.pushBack(15);
	p3.pushBack(16);
	p3.pushBack(17);
	p3.pushBack(18);
	p3.pushBack(19);
	e.setPath(0, std::make_shared<const Path>(p3));

	CHECK(max.getCost(e) == 5);

	Path p4;
	p4.pushBack(15);
	p4.pushBack(16);
	p4.pushBack(17);
	p4.pushBack(18);
	p4.pushBack(19);
	p4.pushBack(20);
	e.pushBack(std::make_shared<const Path>(p4));

	CHECK(max.getCost(e) == 6);
}

TEST_CASE("Testing of SumObjective class") {
	Execution e;

	SumObjective sum;

	CHECK(sum.getCost(e) == 0);

	Path p1;
	p1.pushBack(1);
	p1.pushBack(2);
	p1.pushBack(3);
	p1.pushBack(4);
	e.pushBack(std::make_shared<const Path>(p1));

	CHECK(sum.getCost(e) == 4);

	Path p2;
	p2.pushBack(9);
	p2.pushBack(8);
	p2.pushBack(7);
	e.pushBack(std::make_shared<const Path>(p2));

	CHECK(sum.getCost(e) == 7);

	Path p3;
	p3.pushBack(15);
	p3.pushBack(16);
	p3.pushBack(17);
	p3.pushBack(18);
	p3.pushBack(19);
	e.setPath(0, std::make_shared<const Path>(p3));

	CHECK(sum.getCost(e) == 8);

	Path p4;
	p4.pushBack(15);
	p4.pushBack(16);
	p4.pushBack(17);
	p4.pushBack(18);
	p4.pushBack(19);
	p4.pushBack(20);
	e.pushBack(std::make_shared<const Path>(p4));

	CHECK(sum.getCost(e) == 14);
}