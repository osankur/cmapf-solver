#include "doctest.h"

#include <Graph.hpp>

TEST_CASE("Testing of ExplicitGraph class") {
	ExplicitGraph eg;
	CHECK(eg.node_count() == 0);
	eg.AddNode(0);
	CHECK(eg.node_count() == 1);
	eg.AddNode(1);
	CHECK(eg.node_count() == 2);
	eg.AddEdge(0, 1);
	CHECK(eg.node_count() == 2);
	CHECK(eg.edge_count() == 1);
	eg.AddEdge(1, 0);
	CHECK(eg.node_count() == 2);
	CHECK(eg.edge_count() == 2);
	eg.AddEdge(1, 0);
	CHECK(eg.node_count() == 2);
	CHECK(eg.edge_count() == 2);
	eg.AddEdge(1, 1);
	CHECK(eg.node_count() == 2);
	CHECK(eg.edge_count() == 3);
	eg.AddEdge(0, 0);
	CHECK(eg.node_count() == 2);
	CHECK(eg.edge_count() == 4);
	CHECK(eg.get_neighbors(0).size() == 2);
	CHECK(eg.get_neighbors(1).size() == 2);
}