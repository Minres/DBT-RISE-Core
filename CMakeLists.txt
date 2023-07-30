cmake_minimum_required(VERSION 3.12)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

project(dbt-rise-core VERSION 1.0.0)

include(GNUInstallDirs)
include(add_whole_library)

find_package(Boost COMPONENTS serialization thread REQUIRED)
find_package(tcc QUIET)
if(tcc_FOUND)
    set(WITH_TCC ON)
elseif(TARGET CONAN_PKG::tcc)
	set(WITH_TCC ON)  
else()
    set(WITH_TCC OFF)
endif()

find_package(fmt QUIET)

if(WITH_LLVM)
	if(DEFINED ENV{LLVM_HOME})
		find_path (LLVM_DIR LLVM-Config.cmake $ENV{LLVM_HOME}/lib/cmake/llvm)
	endif()
	find_package(LLVM QUIET CONFIG)
	if(LLVM_FOUND)
		message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
		message(STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")
		llvm_map_components_to_libnames(llvm_libs support core mcjit x86codegen x86asmparser)
	else()
		find_package(LLVM REQUIRED LLVMSupport LLVMCore LLVMMCJIT LLVMX86CodeGen LLVMX86AsmParser)
	endif()
endif()

set(LIB_NAME ${PROJECT_NAME}_a)
set(LIB_SOURCES
	src/iss/debugger/gdb_session.cpp
	src/iss/debugger/encoderdecoder.cpp
	src/iss/debugger/target_adapter_base.cpp
	src/iss/debugger/cmdhandler.cpp
	src/iss/debugger/server_base.cpp
	src/iss/vm_base.cpp
)
if (UNIX)
    list(APPEND LIB_SOURCES  src/iss/plugin/loader.cpp)
endif ()
if(WITH_LLVM)
    list(APPEND LIB_SOURCES src/iss/llvm/jit_helper.cpp src/iss/llvm/vm_base.cpp)
endif()
if(WITH_TCC)
    list(APPEND LIB_SOURCES src/iss/tcc/jit_helper.cpp)
endif()

add_library(${LIB_NAME} STATIC ${LIB_SOURCES})
target_include_directories(${LIB_NAME} PUBLIC src) 
if (MSVC)
    target_compile_definitions(${LIB_NAME} PUBLIC NOMINMAX)
endif()

if(WITH_LLVM)
	target_compile_definitions(${LIB_NAME} PUBLIC ${LLVM_DEFINITIONS})
	target_include_directories(${LIB_NAME} PRIVATE ${LLVM_INCLUDE_DIRS})
	target_link_libraries( ${LIB_NAME} PUBLIC ${LLVM_LIBRARIES})
endif()
if(TARGET Boost::thread)
    target_link_libraries(${LIB_NAME} PUBLIC Boost::thread)
else() 
    target_include_directories(${LIB_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
    target_link_libraries(${LIB_NAME} PUBLIC ${Boost_thread_LIBRARY})
endif()
if(TARGET Boost::serialization)
    target_link_libraries(${LIB_NAME} PUBLIC Boost::serialization)
else() 
    target_include_directories(${LIB_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
    target_link_libraries(${LIB_NAME} PUBLIC ${Boost_serialization_LIBRARY})
endif()
if(TARGET CONAN_PKG::scc)
    target_link_libraries(${LIB_NAME} PUBLIC CONAN_PKG::scc)
else()  
    target_link_libraries(${LIB_NAME} PUBLIC scc-util)
endif()
if(WITH_TCC)
    if(TARGET CONAN_PKG::tcc)
        target_compile_definitions(${LIB_NAME} PUBLIC HAS_TCC)
        target_link_libraries(${LIB_NAME} PUBLIC CONAN_PKG::tcc)
    elseif(TARGET tcc::tcc)
        target_compile_definitions(${LIB_NAME} PUBLIC HAS_TCC)
        target_link_libraries(${LIB_NAME} PUBLIC tcc::tcc)
    else()
        message(FATAL_ERROR "No TCC library found, maybe a find_package() call is missing")
    endif()
endif()
if(TARGET CONAN_PKG::fmt)
    target_compile_definitions(${LIB_NAME} PUBLIC FMT_HEADER_ONLY FMT_USE_WINDOWS_H=0)
    target_link_libraries(${LIB_NAME} PUBLIC CONAN_PKG::fmt)
elseif(TARGET fmt::fmt)
    target_compile_definitions(${LIB_NAME} PUBLIC FMT_HEADER_ONLY FMT_USE_WINDOWS_H=0)
    target_link_libraries(${LIB_NAME} PUBLIC fmt::fmt)
else()
    message(FATAL_ERROR "No FMT library found, maybe a find_package() call is missing")
endif()
target_link_libraries(${LIB_NAME} PUBLIC ${CMAKE_DL_LIBS})

set(LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/dbt_rise_common.h)
set_target_properties(${LIB_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  FRAMEWORK FALSE
  PUBLIC_HEADER "${LIB_HEADERS}"
)

install(TARGETS ${LIB_NAME} COMPONENT ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets                         # for downstream dependencies
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/static
  FRAMEWORK DESTINATION ${CMAKE_INSTALL_LIBDIR}         # for mac
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # headers for mac (note the different component -> different package)
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}      # headers
)

add_whole_library(${PROJECT_NAME} ${LIB_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/iss COMPONENT ${PROJECT_NAME}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # target directory
        FILES_MATCHING # install only matched files
        PATTERN "*.h" # select header files
        )
