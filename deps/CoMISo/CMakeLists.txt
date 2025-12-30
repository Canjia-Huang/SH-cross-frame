add_library(${PROJECT_NAME}_COMISO)

target_include_directories(${PROJECT_NAME}_COMISO
        PRIVATE
        "${CMAKE_SOURCE_DIR}/deps"
        ${EIGEN3_INCLUDE_DIR}
)

# Add sources to the library
target_sources(${PROJECT_NAME}_COMISO
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Solver/EigenLDLTSolver.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/Solver/MISolver.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/Solver/ConstrainedSolver.cc
)

target_link_libraries(${PROJECT_NAME}_COMISO
        PUBLIC
        lapack
        blas
)