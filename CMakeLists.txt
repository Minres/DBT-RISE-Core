cmake_minimum_required(VERSION 3.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../cmake) # main (top) cmake dir
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake) # project specific cmake dir

# CMake useful variables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib") 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Set the name of your project here
project("dbt-core")

# Set the version number of your project here (format is MAJOR.MINOR.PATCHLEVEL - e.g. 1.0.0)
set(VERSION_MAJOR "1")
set(VERSION_MINOR "0")
set(VERSION_PATCH "0")
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

include(Common)

if(DEFINED ENV{LLVM_HOME})
find_path (LLVM_DIR LLVM-Config.cmake $ENV{LLVM_HOME}/lib/cmake/llvm)
endif(DEFINED ENV{LLVM_HOME})

find_package(LLVM REQUIRED CONFIG)

# This sets the include directory for the reference project. This is the -I flag in gcc.
include_directories(
    ${PROJECT_SOURCE_DIR}/incl
    ${PROJECT_SOURCE_DIR}/../sc-components/incl
    ${LLVM_INCLUDE_DIRS}
)

add_dependent_header(util)

add_definitions(${LLVM_DEFINITIONS})

add_subdirectory(src)
