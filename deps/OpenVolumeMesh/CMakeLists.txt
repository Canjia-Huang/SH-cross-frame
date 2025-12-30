add_library(${PROJECT_NAME}_OPENVOLUMEMESH)

target_include_directories(${PROJECT_NAME}_OPENVOLUMEMESH
        PRIVATE
        "${CMAKE_SOURCE_DIR}/deps"
)

target_compile_definitions(${PROJECT_NAME}_OPENVOLUMEMESH
        PRIVATE
        -DINCLUDE_TEMPLATES
)

# Configure sources
file(GLOB_RECURSE SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

# Add sources to the library
target_sources(${PROJECT_NAME}_OPENVOLUMEMESH
        PRIVATE
        ${SOURCE}
)