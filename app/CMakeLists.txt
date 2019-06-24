add_executable(quadcopter main.cpp)

target_link_libraries(quadcopter
    PRIVATE hal
)

toolchain_setTargetOutputPath(quadcopter ${PROJECT_BINARY_DIR}/bin)
toolchain_showTargetConfig(quadcopter)
