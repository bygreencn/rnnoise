set(RNNOISE_EXAMPLE
    rnnoise_demo.c)

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
include_directories("../include/")
include_directories("../unittest-tool/")

link_directories("${PROJECT_BINARY_DIR}/libs")
add_executable(rnnoise_demo ${RNNOISE_EXAMPLE})
add_dependencies(rnnoise_demo rnnoise)
target_link_libraries(rnnoise_demo rnnoise)