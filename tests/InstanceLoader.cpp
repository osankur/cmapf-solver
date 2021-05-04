#include "AppConfig.h"
#include "doctest.h"

#include <InstanceLoader.hpp>

TEST_CASE("Testing of XMLInstanceLoader class") {
	instance::XMLInstanceLoader il(std::string(PROJECT_SOURCE_DIR) + "/tests/assets/Test1.exp", std::string(PROJECT_SOURCE_DIR) + "/tests/assets/");
	il.load();

	CHECK(il.instance().graph().movement().getNodeCount() == 1750);
	CHECK(il.instance().graph().movement().getEdgeCount() == 3168*2 + 1750);
	CHECK(il.instance().graph().communication().getNodeCount() == 1750);
	CHECK(il.instance().graph().communication().getEdgeCount() == 19494 * 2 + 1750);

	CHECK(il.instance().nbAgents() == 4);

	CHECK(il.instance().start().size() == 4);
	CHECK(il.instance().start()[0] == 639);
	CHECK(il.instance().start()[1] == 640);
	CHECK(il.instance().start()[2] == 642);
	CHECK(il.instance().start()[3] == 637);

	CHECK(il.instance().goal().size() == 4);
	CHECK(il.instance().goal()[0] == 362);
	CHECK(il.instance().goal()[1] == 360);
	CHECK(il.instance().goal()[2] == 402);
	CHECK(il.instance().goal()[3] == 320);
	
}