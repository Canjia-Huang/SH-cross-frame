add_library(${PROJECT_NAME}_ALGLIB)

# configure sources
file(GLOB SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

# add sources to the library
target_sources(${PROJECT_NAME}_ALGLIB
        PRIVATE
        ${SOURCE}
)