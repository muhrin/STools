

cmake_minimum_required(VERSION 2.6)

set(STOOLS_VERSION_MAJOR 0)
set(STOOLS_VERSION_MINOR 3)
set(STOOLS_VERSION_PATCH 0)
set(STOOLS_VERSION "${STOOLS_VERSION_MAJOR}.${STOOLS_VERSION_MINOR}.${STOOLS_VERSION_PATCH}")

message(STATUS "Configuring STools ${STOOLS_VERSION}")

project(STools)

## Configuration options ###############################

# Custom paths to libraries #########
set(BOOSTOOLS_ROOT "" CACHE FILEPATH "Path to boost library")
set(ARMADILLO_ROOT "" CACHE FILEPATH "Path to armadillo linear algebra library")
set(QHULL_ROOT "" CACHE FILEPATH "Path to qhull library")


## End configuration options ###########################


# Tell CMake where to find modules
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/;${CMAKE_MODULE_PATH}")

configure_file(
  "${PROJECT_SOURCE_DIR}/src/SToolsConfig.h.in"
  "${PROJECT_BINARY_DIR}/src/SToolsConfig.h"
)



## FIND PACKAGES ########################################
# First add any user specified paths to the package search array
set(CMAKE_PREFIX_PATH "${BOOSTOOLS_ROOT}" "${ARMADILLO_ROOT}")


# Boost #
# Tell Windows builds to look for static boost libraries
if(WIN32)
  set(Boost_USE_STATIC_LIBS ON)
endif(WIN32)
find_package(Boost 1.36.0 COMPONENTS system filesystem program_options REQUIRED)
# Armadillo #
find_package(Armadillo)
if("${ARMADILLO_INCLUDE_DIRS}" STREQUAL "")
  set(ARMADILLO_INCLUDE_DIRS "${ARMADILLO_ROOT}/include")
endif()

## END FIND PACKAGES ####################################

add_subdirectory(lib/spipe)	# StructurePipe

# Include directories with header files
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}/src  # So that we find SToolsConfig.h
)

add_subdirectory(src ${CMAKE_BINARY_DIR}/bin)

set_property(TARGET PROPERTY PROJECT_LABEL "STools")
