# according to https://github.com/horance-liu/flink.cmake/tree/master
# SPDX-License-Identifier: Apache-2.0

include(CMakeParseArguments)

macro(add_whole_library target_lib source_lib)
  get_target_property(SOURCE_LIBS ${source_lib} INTERFACE_LINK_LIBRARIES)
  message("add_whole_library: INTERFACE_LINK_LIBRARIES=${SOURCE_LIBS}")
  get_target_property(SOURCE_INCL ${source_lib} INTERFACE_INCLUDE_DIRECTORIES)
  message("add_whole_library: INTERFACE_INCLUDE_DIRECTORIES=${SOURCE_INCL}")

  add_library(${target_lib} INTERFACE)
  if(MSVC)
	set_property(TARGET ${target_lib} 
	  PROPERTY INTERFACE_LINK_LIBRARIES "/WHOLEARCHIVE:${source_lib}")
    target_link_libraries(${target} ${visibility} "/WHOLEARCHIVE:${lib}")
  elseif(APPLE)
	set_property(TARGET ${target_lib} 
	  PROPERTY INTERFACE_LINK_LIBRARIES -Wl,-force_load,${source_lib})
  else()
	set_property(TARGET ${target_lib} 
	  PROPERTY INTERFACE_LINK_LIBRARIES -Wl,--whole-archive,$<TARGET_FILE:${source_lib}>,--no-whole-archive)
  endif()

  target_include_directories(${target_lib} INTERFACE ${SOURCE_INCL})
  target_link_libraries(${target_lib} INTERFACE ${SOURCE_LIBS})

endmacro()