cmake_minimum_required(VERSION 3.10)


idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")

target_include_directories(${COMPONENT_TARGET} PRIVATE
    ${CMAKE_SOURCE_DIR}/components/board_config/include
)


target_link_libraries(${COMPONENT_TARGET} PRIVATE
    # TO DO UNCOMMENT THE LINES BELOW WHEN YOU HAVE THESE COMPONENTS
    # motor_control
    # camera_driver
    # bluetooth_remote
    )

