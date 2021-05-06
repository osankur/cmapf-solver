if(TOGGLE_BOOST_USE_STATIC)
    SET(Boost_USE_STATIC_LIBS ON)
else()
    ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
endif()

find_package(Boost 1.71.0 COMPONENTS log REQUIRED)