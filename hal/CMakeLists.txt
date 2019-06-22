if (NOT DEFINED PRODUCT)
    message(FATAL_ERROR "'PRODUCT' is not defined!")
endif ()

add_library(hal EXCLUDE_FROM_ALL
    Hardware.cpp
)

# Product has to be added first, because it configures the required boards and platform implementation.
add_subdirectory(product)
add_subdirectory(board)
add_subdirectory(private)
add_subdirectory(public)

target_link_libraries(hal
    INTERFACE product
    PUBLIC hal_public
    PRIVATE hal_private
)

# Print HAL configuration.
message(STATUS "-------------------------------------------")
message(STATUS "HAL configuration:")
message(STATUS "  product  : ${PRODUCT}")
message(STATUS "  platform : ${PLATFORM}")
string(REPLACE ";" " " BOARDS_STR "${BOARDS}")
message(STATUS "  boards   : ${BOARDS_STR}")
message(STATUS "-------------------------------------------")
