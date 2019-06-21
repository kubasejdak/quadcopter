add_executable(quadcopter main.cpp)

target_link_libraries(quadcopter
    PRIVATE hal
)
