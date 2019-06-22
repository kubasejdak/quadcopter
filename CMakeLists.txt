cmake_minimum_required(VERSION 3.13.4)
#include(tools/cmake/all.cmake)

#utils_getVersion(VERSION)
set(VERSION 0.1.0)
project(quadcopter VERSION ${VERSION} LANGUAGES CXX)

# Project options.
option(QUADCOPTER_COVERAGE "Generate coverage report" OFF)
option(QUADCOPTER_TESTS "Build unit tests" OFF)
option(QUADCOPTER_DOCS "Generate doxygen documentation" OFF)

# Common compilation flags.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic -Werror)

#if(QUADCOPTER_COVERAGE)
#    utils_log("Building quadcopter with coverage report")
#    set(COVERAGE_IGNORE '${PROJECT_SOURCE_DIR}/test/*' '${PROJECT_SOURCE_DIR}/lib/*')
#    coverage_addTarget(${COVERAGE_IGNORE})
#endif()

add_subdirectory(app)
add_subdirectory(hal)

#if(QUADCOPTER_TESTS)
#    toolchain_showConfig()
#    utils_log("Building quadcopter with tests")
#    enable_testing()
#    add_subdirectory(test)
#endif()

#if(QUADCOPTER_DOCS)
#    utils_log("Building quadcopter with docs")
#    doxygen_addTarget(${PROJECT_SOURCE_DIR})
#endif()
