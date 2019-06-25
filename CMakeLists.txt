cmake_minimum_required(VERSION 3.13.4)
include(tools/cmake/all.cmake)

utils_getVersion(VERSION)
project(quadcopter VERSION ${VERSION} LANGUAGES CXX)

# Add conan files.
if (EXISTS ${PROJECT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${PROJECT_BINARY_DIR}/conanbuildinfo.cmake)
    conan_basic_setup()
endif ()

# Configure external libraries.
find_package(Boost 1.70)
if (NOT Boost_FOUND)
    utils_error("Boost not found")
endif ()
find_package(CLI11 CONFIG)
if (NOT CLI11_FOUND)
    utils_error("CLI11 not found")
endif ()

# Project options.
option(QUADCOPTER_COVERAGE "Generate coverage report" OFF)
option(QUADCOPTER_TESTS "Build unit tests" OFF)
option(QUADCOPTER_DOCS "Generate doxygen documentation" OFF)

if (NOT PRODUCT)
    set(PRODUCT quadcopter_rpi)
endif ()

# Common compilation flags.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic -Werror)

if (QUADCOPTER_COVERAGE)
    utils_log("Building quadcopter with coverage report")
    set(COVERAGE_IGNORE '${PROJECT_SOURCE_DIR}/test/*' '${PROJECT_SOURCE_DIR}/lib/*')
    coverage_addTarget(${COVERAGE_IGNORE})
endif ()

toolchain_showConfig()

add_subdirectory(app)
add_subdirectory(hal)

if (QUADCOPTER_TESTS)
    utils_log("Building quadcopter with tests")
    enable_testing()
    add_subdirectory(test)
endif ()

if (QUADCOPTER_DOCS)
    utils_log("Building quadcopter with docs")
    doxygen_addTarget(${PROJECT_SOURCE_DIR})
endif ()
