cmake_minimum_required(VERSION 3.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

# Set the name of your project here
project("dbt-core")

# Set the version number of your project here (format is MAJOR.MINOR.PATCHLEVEL - e.g. 1.0.0)
set(VERSION_MAJOR "0")
set(VERSION_MINOR "9")
set(VERSION_PATCH "27")
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

# Build config.h
#configure_file ( ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/tinycc/config.h )

# Detect native platform
if(WIN32)
  set(BUILD_SHARED_LIBS ON)
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "ARM")
    set(TCC_NATIVE_TARGET "WinCE")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(TCC_NATIVE_TARGET "Win64")
  else()
    set(TCC_NATIVE_TARGET "Win32")
  endif()
else()
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
    set(TCC_NATIVE_TARGET "ARM")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^armv5")
      set(TCC_ARM_VERSION_DEFAULT 5)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^armv6")
      set(TCC_ARM_VERSION_DEFAULT 6)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^armv7")
      set(TCC_ARM_VERSION_DEFAULT 7)
    else()
      set(TCC_ARM_VERSION_DEFAULT 4)
    endif()
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(TCC_NATIVE_TARGET "x86_64")
    set(TCC_ARCH_DIR "x86_64-linux-gnu")
  else()
    set(TCC_NATIVE_TARGET "i386")
    set(TCC_ARCH_DIR "i386-linux-gnu")
  endif()
endif()


set(SRC_FILES tinycc/tcc.c tinycc/tcctools.c tinycc/libtcc.c tinycc/tccpp.c tinycc/tccgen.c tinycc/tccelf.c tinycc/tccasm.c tinycc/tccrun.c tinycc/tcc.h tinycc/config.h tinycc/libtcc.h tinycc/tcctok.h)
if(${TCC_NATIVE_TARGET} STREQUAL "i386")
	set(SRC_FILES         ${SRC_FILES} i386-gen.c i386-link.c i386-asm.c i386-asm.h i386-tok.h)
	set(DEFINES  TCC_TARGET_I386)
	if(WIN32)
		set(i386-SRC_FILES   ${SRC_FILES} tccpe.c)
		set(DEFINES  ${DEFINES} TCC_TARGET_PE)
	endif()
elseif(${TCC_NATIVE_TARGET} STREQUAL "x86_64")
	set(SRC_FILES       ${SRC_FILES} tinycc/x86_64-gen.c tinycc/x86_64-link.c tinycc/i386-asm.c tinycc/x86_64-asm.h)
	set(DEFINES  TCC_TARGET_X86_64) #should also have CONFIG_TRIPLET="\"x86_64-linux-gnu\""
	if(WIN32)
		set(SRC_FILES ${SRC_FILES} tccpe.c)
		set(DEFINES  ${DEFINES} TCC_TARGET_PE)
	else(APPLE)
		set(DEFINES  ${DEFINES} DTCC_TARGET_MACHO)
	endif()
elseif(${TCC_NATIVE_TARGET} STREQUAL "arm")
	set(SRC_FILES          ${SRC_FILES} arm-gen.c arm-link.c arm-asm.c)
	set(SRC_FILES        ${SRC_FILES} arm64-gen.c arm64-link.c)
endif()

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/tinycc/config.h
    COMMAND sh ./configure
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tinycc
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tinycc/configure
    COMMENT "Run configure for TinyCC"
    RESULT_VARIABLE res
    VERBATIM
)
if(${res})
    message(FATAL_ERROR,"Cannot run configure")
endif()
add_custom_target(tinyccconfig DEPENDS ${tinycc/config.h})

# Set C compiler flags.
include(CheckCCompilerFlag)

check_c_compiler_flag ( -fno-strict-aliasing, HAVE_NOSTRICTALIASING )
if ( HAVE_NOSTRICTALIASING )
  add_definitions ( -fno-strict-aliasing )
endif()
add_definitions(
 -Wno-unused-variable 
 -Wdeclaration-after-statement 
 -fno-strict-aliasing 
 -Wno-pointer-sign 
 -Wno-sign-compare 
 -Wno-unused-result)

# Define a variable in order not to repeat ourselves.
set(LIBRARY_NAME tinycc)
# Define the library
add_library(${LIBRARY_NAME} STATIC ${SRC_FILES})
set_property(TARGET ${LIBRARY_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(${LIBRARY_NAME} PRIVATE ONE_SOURCE=0 ${DEFINES})
target_include_directories(${LIBRARY_NAME} PUBLIC tinycc)
target_link_libraries(${LIBRARY_NAME} PUBLIC ${CMAKE_DL_LIBS})
add_dependencies(${LIBRARY_NAME} tinyccconfig)
