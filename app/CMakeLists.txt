utils_log("Configuring '${CMAKE_CURRENT_SOURCE_DIR}/version.hpp'")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/version.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/version.hpp)

add_executable(quadcopter main.cpp)

target_link_libraries(quadcopter
    PRIVATE CLI11::CLI11 hal utils spdlog::spdlog
)

toolchain_setTargetOutputPath(quadcopter ${PROJECT_BINARY_DIR}/bin)
toolchain_showTargetConfig(quadcopter)
