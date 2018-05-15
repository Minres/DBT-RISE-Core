FILE(GLOB IssHeaders *.h)
set(LIB_HEADERS ${IssHeaders} )
set(LIB_SOURCES
    dbgsrvbase.cpp
    gdb_session.cpp
    jit_helper.cpp
    vm_base.cpp
    cmdhandler.cpp
    target_adapter_base.cpp
    encoderdecoder.cpp
)

# Define two variables in order not to repeat ourselves.
set(LIBRARY_NAME dbt-core)

# Define the library
#add_library(${LIBRARY_NAME} SHARED ${LIB_SOURCES})
add_library(${LIBRARY_NAME} ${LIB_SOURCES})
#add_library(${LIBRARY_NAME} ${LIB_SOURCES})
SET(${LIBRARY_NAME} -Wl,-whole-archive -l${LIBRARY_NAME} -Wl,-no-whole-archive)
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
  ARCHIVE DESTINATION lib COMPONENT libs   # static lib
  LIBRARY DESTINATION lib COMPONENT libs   # shared lib
  FRAMEWORK DESTINATION bin COMPONENT libs # for mac
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME} COMPONENT devel   # headers for mac (note the different component -> different package)
  INCLUDES DESTINATION incl                # headers
)

