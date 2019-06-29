add_library(gpio
    BroadcomGpioMem.cpp
    )

target_include_directories(gpio
    PUBLIC .
    )

target_link_libraries(gpio
    PUBLIC hal_public hal_private
    )
