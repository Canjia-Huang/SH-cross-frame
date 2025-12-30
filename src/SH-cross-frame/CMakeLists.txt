add_library(${PROJECT_NAME}_SH_CROSS_FRAME)

target_include_directories(${PROJECT_NAME}_SH_CROSS_FRAME
        PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps"
        ${EIGEN3_INCLUDE_DIR}
)

# Configure sources
file(GLOB SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

# Add sources to the library
target_sources(${PROJECT_NAME}_SH_CROSS_FRAME
        PRIVATE
        ${SOURCE}
)

# Add libraries
target_link_libraries(${PROJECT_NAME}_SH_CROSS_FRAME
        PRIVATE
        ${PROJECT_NAME}_COMISO
        ${PROJECT_NAME}_ALGLIB
        ${PROJECT_NAME}_OPENVOLUMEMESH
)