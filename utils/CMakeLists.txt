add_library(utils
    Error.cpp
    logger.cpp
)

target_include_directories(utils
    PUBLIC include/
)

target_link_libraries(utils
    PRIVATE spdlog::spdlog
)
