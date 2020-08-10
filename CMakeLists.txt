cmake_minimum_required(VERSION 3.12)

project("dbt-core" VERSION 1.0.0)

include(GNUInstallDirs)

find_package(Boost 1.70 COMPONENTS thread REQUIRED)

if(WITH_LLVM)
	if(DEFINED ENV{LLVM_HOME})
		find_path (LLVM_DIR LLVM-Config.cmake $ENV{LLVM_HOME}/lib/cmake/llvm)
	endif(DEFINED ENV{LLVM_HOME})
	find_package(LLVM REQUIRED CONFIG)
	message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
	message(STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")
	llvm_map_components_to_libnames(llvm_libs support core mcjit x86codegen x86asmparser)
endif()

set(LIB_SOURCES
    src/dbgsrvbase.cpp
    src/gdb_session.cpp
    src/vm_base.cpp
    src/tcc_jit_helper.cpp
    src/cmdhandler.cpp
    src/target_adapter_base.cpp
    src/encoderdecoder.cpp
)

add_library(dbt-core ${LIB_SOURCES})
if(WITH_LLVM)
	target_compile_definitions(dbt-core PUBLIC ${LLVM_DEFINITIONS})
	target_include_directories(dbt-core PRIVATE ${LLVM_INCLUDE_DIRS}) 
endif()
target_include_directories(dbt-core PUBLIC ${PROJECT_SOURCE_DIR}/incl) 
target_include_directories(dbt-core PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(dbt-core PRIVATE scc-util)
target_link_libraries(dbt-core PUBLIC CONAN_PKG::tcc CONAN_PKG::fmt ${Boost_LIBRARIES} ${CMAKE_DL_LIBS})
set_target_properties(dbt-core PROPERTIES
  VERSION ${PROJECT_VERSION}
  FRAMEWORK FALSE
  PUBLIC_HEADER "${LIB_HEADERS}"
)

install(TARGETS dbt-core
  EXPORT ${PROJECT_NAME}Targets            # for downstream dependencies
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT libs   # static lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT libs   # binaries
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT libs   # shared lib
  FRAMEWORK DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT libs # for mac
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT devel   # headers for mac (note the different component -> different package)
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}                # headers
)
