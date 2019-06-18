cmake_minimum_required(VERSION 3.8)
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
endif()

find_package(LLVM REQUIRED CONFIG)

set(LIB_SOURCES
    src/dbgsrvbase.cpp
    src/gdb_session.cpp
    src/jit_helper.cpp
    src/vm_base.cpp
    src/cmdhandler.cpp
    src/target_adapter_base.cpp
    src/encoderdecoder.cpp
)

# Define two variables in order not to repeat ourselves.
set(LIBRARY_NAME dbt-core)
# Define the library
add_library(${LIBRARY_NAME} ${LIB_SOURCES})

target_compile_definitions(${LIBRARY_NAME} PUBLIC ${LLVM_DEFINITIONS})
target_include_directories(${LIBRARY_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/incl) 
target_include_directories(${LIBRARY_NAME} PRIVATE ${LLVM_INCLUDE_DIRS}) 

#set_target_properties(foo PROPERTIES LINK_FLAGS "My lib link flags")
#set_target_properties(exe PROPERTIES LINK_FLAGS "My exe link flags")
#target_link_options(${LIBRARY_NAME} BEFORE INTERFACE -Wl,-whole-archive INTERFACE -Wl,-no-whole-archive)

target_link_libraries(${LIBRARY_NAME} PRIVATE scc-util)

# Set the build version. It will be used in the name of the lib, with corresponding
# symlinks created. SOVERSION could also be specified for api version.
set_target_properties(${LIBRARY_NAME} PROPERTIES
  VERSION ${VERSION}
  FRAMEWORK FALSE
  PUBLIC_HEADER "${LIB_HEADERS}"
)

# Says how and where to install software
# Targets:
#   * <prefix>/lib/<libraries>
#   * header location after install: <prefix>/include/<project>/*.h
#   * headers can be included by C++ code `#<project>/Bar.hpp>`
install(TARGETS ${LIBRARY_NAME}
  EXPORT ${PROJECT_NAME}Targets            # for downstream dependencies
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib COMPONENT libs   # shared lib
  ARCHIVE DESTINATION lib COMPONENT libs   # static lib
  FRAMEWORK DESTINATION bin COMPONENT libs # for mac
  PUBLIC_HEADER DESTINATION include COMPONENT devel   # headers for mac (note the different component -> different package)
  INCLUDES DESTINATION incl                # headers
)
