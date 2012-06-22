

cmake_minimum_required(VERSION 2.6)

set(ST_VERSION_MAJOR 0)
set(ST_VERSION_MINOR 1)

message(STATUS "Configuring STools ${ST_VERSION_MAJOR}.${ST_VERSION_MINOR}")

project(STools)

if(NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeLocal.txt)
  message(STATUS "Generating local configuration file, CMakeLocal.txt, for this build.  Try changing settings here if there are any problems building.")
  configure_file(
    "${PROJECT_SOURCE_DIR}/CMakeLocal.txt.in"
    "${CMAKE_BINARY_DIR}/CMakeLocal.txt"
  )
endif()

# Include local settings
include("${CMAKE_BINARY_DIR}/CMakeLocal.txt")

# Tell CMake where to find modules
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/;${CMAKE_MODULE_PATH}")

configure_file (
  "${PROJECT_SOURCE_DIR}/src/SToolsConfig.h.in"
  "${PROJECT_BINARY_DIR}/src/SToolsConfig.h"
  )



## FIND PACKAGES ########################################
# First add any user specified paths to the package search array
set(CMAKE_PREFIX_PATH "${BOOST_ROOT}" "${ARMADILLO_ROOT}")


# Boost #
# Tell Windows builds to look for static boost libraries
if(WIN32)
  set(Boost_USE_STATIC_LIBS ON)
endif(WIN32)
find_package(Boost 1.36.0 COMPONENTS system filesystem REQUIRED)
# Armadillo #
find_package(Armadillo)
if("${ARMADILLO_INCLUDE_DIRS}" STREQUAL "")
  set(ARMADILLO_INCLUDE_DIRS "${ARMADILLO_ROOT}/include")
endif()

## END FIND PACKAGES ####################################

# Include directories with header files
  include_directories(
    ${Boost_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
  )

add_subdirectory(lib/pipelib)   # Pipeline library
add_subdirectory(lib/sslib)     # Structure searching library
add_subdirectory(lib/yaml-cpp)  # YAML library
add_subdirectory(lib/spipe)	# StructurePipe
add_subdirectory(src ${CMAKE_BINARY_DIR}/bin)

set_property(TARGET PROPERTY PROJECT_LABEL "STools")
