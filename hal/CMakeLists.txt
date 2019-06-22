add_library(hal EXCLUDE_FROM_ALL
    Hardware.cpp
)

add_subdirectory(private)
add_subdirectory(public)

target_link_libraries(hal
    PUBLIC hal_public
    PRIVATE hal_private
)
