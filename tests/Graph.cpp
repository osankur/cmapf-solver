#include "doctest.h"

#include <Graph.hpp>

TEST_CASE("Testing of ExplicitGraph class") {
	ExplicitGraph eg;
	CHECK(eg.getNodeCount() == 0);
	eg.addNode(0);
	CHECK(eg.getNodeCount() == 1);
	eg.addNode(1);
	CHECK(eg.getNodeCount() == 2);
	eg.addEdge(0, 1);
	CHECK(eg.getNodeCount() == 2);
	CHECK(eg.getEdgeCount() == 1);
	eg.addEdge(1, 0);
	CHECK(eg.getNodeCount() == 2);
	CHECK(eg.getEdgeCount() == 2);
	eg.addEdge(1, 0);
	CHECK(eg.getNodeCount() == 2);
	CHECK(eg.getEdgeCount() == 2);
	eg.addEdge(1, 1);
	CHECK(eg.getNodeCount() == 2);
	CHECK(eg.getEdgeCount() == 3);
	eg.addEdge(0, 0);
	CHECK(eg.getNodeCount() == 2);
	CHECK(eg.getEdgeCount() == 4);
	CHECK(eg.getNeighbors(0).size() == 2);
	CHECK(eg.getNeighbors(1).size() == 2);
}