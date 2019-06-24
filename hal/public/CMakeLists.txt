add_library(hal_public EXCLUDE_FROM_ALL
    Error.cpp
    Device.cpp
)

target_include_directories(hal_public
    PUBLIC include/
)

target_link_libraries(hal_public
    PRIVATE hal_private
)
