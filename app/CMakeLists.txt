#########################################################################################################################
## main
#########################################################################################################################
#add_executable(${PROJECT_NAME}_MAIN
#        test_simple.cc
#)
#
#target_include_directories(${PROJECT_NAME}_MAIN
#        PRIVATE
#        "${CMAKE_SOURCE_DIR}/deps"
#        "${CMAKE_SOURCE_DIR}/src"
#        ${EIGEN3_INCLUDE_DIR}
#)
#
#target_link_libraries(${PROJECT_NAME}_MAIN
#        PRIVATE
#        ${PROJECT_NAME}_SH_CROSS_FRAME
#        ${PROJECT_NAME}_COMISO
#        ${PROJECT_NAME}_ALGLIB
#        ${PROJECT_NAME}_OPENVOLUMEMESH
#)