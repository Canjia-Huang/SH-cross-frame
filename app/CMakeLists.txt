#########################################################################################################################
## CLI
#########################################################################################################################
add_executable(${PROJECT_NAME}_CLI
        main_CLI.cpp
)

target_include_directories(${PROJECT_NAME}_CLI
        PRIVATE
        "${CMAKE_SOURCE_DIR}/deps"
        "${CMAKE_SOURCE_DIR}/src"
        ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}_CLI
        PRIVATE
        ${PROJECT_NAME}_SH_CROSS_FRAME
        Geogram::geogram
)