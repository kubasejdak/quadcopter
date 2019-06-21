add_library(hal INTERFACE
)

add_subdirectory(public)

target_link_libraries(hal
    INTERFACE hal_public
)
