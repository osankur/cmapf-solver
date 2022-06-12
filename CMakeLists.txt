# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

project ("cmapf-solver" VERSION 0.1)

option(ENABLE_WARNINGS_SETTINGS "Allow target_set_warnings to add flags and defines.
                                 Set this to OFF if you want to provide your own warning parameters." OFF)
option(ENABLE_LTO "Enable link time optimization" ON)
option(ENABLE_DOCTESTS "Include tests in the library. Setting this to OFF will remove all doctest related code.
                        Tests in tests/*.cpp will still be enabled." ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
include(ConfigSafeGuards)
include(Colors)
include(CTest)
include(Doctest)
include(Documentation)
include(LTO)
include(Misc)
include(Warnings)

SET(Boost_USE_STATIC_LIBS ON)
ADD_DEFINITIONS(-DDEBUG)
#ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)

FIND_PACKAGE(Boost 1.71.0 COMPONENTS log program_options REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_library(rapidxml INTERFACE)
target_include_directories(rapidxml INTERFACE ${PROJECT_SOURCE_DIR}/external/rapidxml/)

add_library(magic_enum INTERFACE)
target_include_directories(magic_enum INTERFACE ${PROJECT_SOURCE_DIR}/external/magic_enum/include)

# Check for LTO support.
find_lto(CXX)

add_definitions(-w)

# --------------------------------------------------------------------------------
#                         Locate files (change as needed).
# --------------------------------------------------------------------------------
set(SOURCES          # All .cpp files in src/
    "src/Path.cpp"
    "src/Configuration.cpp"
    "src/Execution.cpp"
    "src/InstanceLoader.cpp"
    "src/Graph.cpp")
set(TESTFILES        # All .cpp files in tests/
    "tests/Path.cpp"
)
set(LIBRARY_NAME engine)  # Default name for the library built from src/*.cpp (change if you wish)

# --------------------------------------------------------------------------------
#                            Build! (Change as needed)
# --------------------------------------------------------------------------------
# Compile all sources into a library.
add_library(${LIBRARY_NAME} OBJECT ${SOURCES})

# Lib needs its header files, and users of the library must also see these (PUBLIC). (No change needed)
target_include_directories(${LIBRARY_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

# There's also (probably) doctests within the library, so we need to see this as well.
target_link_libraries(${LIBRARY_NAME} PUBLIC doctest)
target_link_libraries(${LIBRARY_NAME} PUBLIC rapidxml)
target_link_libraries(${LIBRARY_NAME} PUBLIC magic_enum)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} LINK_PUBLIC ${Boost_LIBRARIES} )

# Set the compile options you want (change as needed).
target_set_warnings(${LIBRARY_NAME} ENABLE ALL DISABLE Annoying)
# target_compile_options(${LIBRARY_NAME} ... )  # For setting manually.

# Add an executable for the file app/main.cpp.
# If you add more executables, copy these lines accordingly.
add_executable(main app/main.cpp)   # Name of exec. and location of file.
target_link_libraries(main PRIVATE ${LIBRARY_NAME})  # Link the executable to library (if it uses it).
target_set_warnings(main ENABLE ALL DISABLE Annoying) # Set warnings (if needed).
target_enable_lto(main optimized)  # enable link-time-optimization if available for non-debug configurations

# Set the properties you require, e.g. what C++ standard to use. Here applied to library and main (change as needed).
set_target_properties(
    ${LIBRARY_NAME} main 
      PROPERTIES 
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES 
        CMAKE_CXX20_STANDARD_COMPILE_OPTION "-std:c++latest"
        CMAKE_CXX20_EXTENSION_COMPILE_OPTION "-std:c++latest"
        CXX_EXTENSIONS NO
)
# Set up tests (see tests/CMakeLists.txt).
#add_subdirectory(tests)

add_executable(console_main app/console_main.cpp)
set_target_properties(console_main PROPERTIES OUTPUT_NAME "cmapf-solver")

target_link_libraries(console_main PRIVATE ${LIBRARY_NAME})  # Link the executable to library (if it uses it).
target_set_warnings(console_main ENABLE ALL DISABLE Annoying) # Set warnings (if needed).
target_enable_lto(console_main optimized)  # enable link-time-optimization if available for non-debug configurations

# Set the properties you require, e.g. what C++ standard to use. Here applied to library and main (change as needed).
set_target_properties(
    ${LIBRARY_NAME} console_main 
      PROPERTIES 
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES 
        CMAKE_CXX20_STANDARD_COMPILE_OPTION "-std:c++latest"
        CMAKE_CXX20_EXTENSION_COMPILE_OPTION "-std:c++latest"
        CXX_EXTENSIONS NO
)
