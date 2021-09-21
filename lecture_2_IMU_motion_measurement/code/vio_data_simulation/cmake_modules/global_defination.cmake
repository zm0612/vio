set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
configure_file (
    ${PROJECT_SOURCE_DIR}/src/global_defination.h.in
    ${PROJECT_BINARY_DIR}/src/global_defination.h)
include_directories(${PROJECT_BINARY_DIR}/src)