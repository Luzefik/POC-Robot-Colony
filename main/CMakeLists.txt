set(srcs)

set(requires "bluepad32" "btstack")

idf_component_register(SRCS "my_platform.c" "main.c" "driving.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")

target_compile_definitions(${COMPONENT_LIB} PRIVATE BLUEPAD32_PLATFORM_CUSTOM)
