########################################################################################################################
# CMake options
########################################################################################################################
cmake_minimum_required(VERSION 3.30)
project(SHCrossFrame)

if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8")
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_definitions(
        -DCOMISO_EIGEN3_AVAILABLE
        -DINCLUDE_TEMPLATES
)


########################################################################################################################
# CMake module path
########################################################################################################################
list(PREPEND CMAKE_MODULE_PATH
        ${CMAKE_CURRENT_LIST_DIR}/cmake)


########################################################################################################################
# Set variables
########################################################################################################################
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin) # Set output path for the compiled files

set(OPTION_BUILD_TESTS ON CACHE BOOL "Build tests (based on GoogleTest).")


########################################################################################################################
# Include 3rdParty
########################################################################################################################
include(Dependencies)


########################################################################################################################
# Add subdirectories
########################################################################################################################
add_subdirectory(deps)
add_subdirectory(src)
add_subdirectory(app)

message(STATUS "ENABLE_BUILD_TESTS = ${OPTION_BUILD_TESTS}")
if(OPTION_BUILD_TESTS)
    add_subdirectory(tests)
endif()