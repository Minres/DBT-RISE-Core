cmake_minimum_required(VERSION 3.12)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
cmake_policy(SET CMP0077 NEW)

project(dbt-rise-core VERSION 1.0.0)

option(WITH_TCC "Build TCC backend" ON)
option(WITH_LLVM "Build LLVM backend" OFF)
option(WITH_ASMJIT "Build ASMJIT backend" ON)


include(GNUInstallDirs)
include(add_whole_library)

find_package(Boost COMPONENTS serialization thread REQUIRED)
find_package(fmt)

if(WITH_ASMJIT)
	find_package(asmjit)
endif()
if(WITH_LLVM)
	find_package(LLVM REQUIRED LLVMSupport LLVMCore LLVMMCJIT LLVMX86CodeGen LLVMX86AsmParser)
endif()

#if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux" AND WITH_TCC)
if(WITH_TCC)
message("Adding TCC")
	include(FetchContent)
	FetchContent_Declare(
	  	tcc_git
	  	GIT_REPOSITORY git://repo.or.cz/tinycc.git
	  	GIT_TAG        16456168430c9e185dd94b8215aa77d02bbb8a2c
	  	UPDATE_DISCONNECTED ON
		CONFIGURE_COMMAND ${tcc_git_SOURCE_DIR}/configure --prefix=${tcc_git_BINARY_DIR}/install
		BUILD_COMMAND make
		BUILD_BYPRODUCTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libtcc.a
	)
	FetchContent_MakeAvailable(tcc_git)
	FetchContent_GetProperties(tcc_git)
	add_library(tcc::tcc STATIC IMPORTED GLOBAL)
	set_property(TARGET tcc::tcc PROPERTY IMPORTED_LOCATION ${tcc_git_BINARY_DIR}/lib/libtcc.a)
	add_dependencies(tcc::tcc tcc_git)	#	add_custom_target(myExternalTarget COMMAND <invoke the repo's build system>)
#	FetchContent_GetProperties(tcc_git)
#	if (NOT tcc_git)
#	    FetchContent_Populate(tcc_git)
#	
#	    # here I would like to declare imported libraries:
#	    add_library(tcc::tcc STATIC IMPORTED GLOBAL)
#	    target_include_directories(tcc::tcc INTERFACE ${tcc_git_SOURCE_DIR})
#	    set_property(TARGET tcc::tcc   PROPERTY IMPORTED_LOCATION ${tcc_git_SOURCE_DIR})
#	
#	endif()
endif()

set(LIB_NAME ${PROJECT_NAME})
set(LIB_SOURCES
	src/iss/debugger/gdb_session.cpp
	src/iss/debugger/encoderdecoder.cpp
	src/iss/debugger/target_adapter_base.cpp
	src/iss/debugger/cmdhandler.cpp
	src/iss/debugger/server_base.cpp
	src/iss/vm_jit_funcs.cpp
	src/iss/plugin/loader.cpp
	src/iss/plugin/caculator.cpp
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
if(WITH_ASMJIT)
    list(APPEND LIB_SOURCES src/iss/asmjit/jit_helper.cpp)
endif()
add_library(${LIB_NAME} STATIC ${LIB_SOURCES})
target_include_directories(${LIB_NAME} PUBLIC src) 
if (MSVC)
    target_compile_definitions(${LIB_NAME} PUBLIC NOMINMAX)
endif()
target_compile_definitions(${LIB_NAME} PRIVATE LIB_EXEC_DIR=${CMAKE_INSTALL_LIBEXECDIR})

if(WITH_LLVM)
	target_compile_definitions(${LIB_NAME} PUBLIC WITH_LLVM ${LLVM_DEFINITIONS})
	target_include_directories(${LIB_NAME} PUBLIC ${LLVM_INCLUDE_DIRS})
	target_link_libraries( ${LIB_NAME} PUBLIC ${LLVM_LIBRARIES})
endif()

if(WITH_TCC)
    target_compile_definitions(${LIB_NAME} PUBLIC WITH_TCC)
    target_link_libraries(${LIB_NAME} PUBLIC tcc)
endif()
if(WITH_ASMJIT)
    target_compile_definitions(${LIB_NAME} PUBLIC WITH_ASMJIT)
    target_link_libraries(${LIB_NAME} PUBLIC asmjit::asmjit)
endif()
target_link_libraries(${LIB_NAME} PUBLIC scc-util)
target_link_libraries(${LIB_NAME} PUBLIC fmt::fmt)
target_link_libraries(${LIB_NAME} PUBLIC Boost::serialization Boost::thread)
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

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/iss COMPONENT ${PROJECT_NAME}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # target directory
        FILES_MATCHING # install only matched files
        PATTERN "*.h" # select header files
        )
