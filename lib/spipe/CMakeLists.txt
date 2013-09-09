

cmake_minimum_required(VERSION 2.6)

set(SP_VERSION_MAJOR 0)
set(SP_VERSION_MINOR 3)
set(SP_VERSION_PATCH 0)
set(SP_VERSION "${SP_VERSION_MAJOR}.${SP_VERSION_MINOR}.${SP_VERSION_PATCH}")

message(STATUS "Configuring StructurePipe ${SP_VERSION}")

project(StructurePipe)

## Configuration options ###############################

# Custom paths to libraries #########
set(BOOST_ROOT "" CACHE FILEPATH "Path to boost library")
set(ARMADILLO_ROOT "" CACHE FILEPATH "Path to armadillo linear algebra library")

# Build settings
set(SP_USE_YAML TRUE CACHE BOOL "Build StructurePipe with YAML support.")
set(SP_ENABLE_TESTING FALSE CACHE BOOL "Build spipe tests.")
set(SP_ENABLE_THREAD_AWARE TRUE CACHE BOOL "Enable multithreaded awareness in spipe.")

configure_file (
  "${PROJECT_SOURCE_DIR}/src/StructurePipeConfig.h.in"
  "${PROJECT_BINARY_DIR}/src/StructurePipeConfig.h"
)

## End configuration options ###########################

# Tell CMake where to find modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

## FIND PACKAGES ########################################
# First add any user specified paths to the package search array
set(CMAKE_PREFIX_PATH
  "${BOOST_ROOT}"
  "${ARMADILLO_ROOT}"
)

## Boost ##
# Disable auto-linking
add_definitions(-DBOOST_ALL_NO_LIB)
find_package(Boost 1.36.0 REQUIRED COMPONENTS system filesystem regex)

## Armadillo ##
if(NOT ARMADILLO_INCLUDE_DIRS)
  find_package(Armadillo CONFIG REQUIRED)
  if("${ARMADILLO_INCLUDE_DIRS}" STREQUAL "")
    set(ARMADILLO_INCLUDE_DIRS "${ARMADILLO_ROOT}/include")
  endif()
endif(NOT ARMADILLO_INCLUDE_DIRS)

## Yaml ##
if(SP_USE_YAML)
  if(NOT YAML_CPP_LIBRARIES)
    find_package(YamlCpp REQUIRED)
  endif(NOT YAML_CPP_LIBRARIES)
endif(SP_USE_YAML)

#
# Spglib
if(NOT SPGLIB_INCLUDE_DIRS)
  find_package(Spglib REQUIRED)
endif(NOT SPGLIB_INCLUDE_DIRS)

## END FIND PACKAGES ####################################


## First libraries #####################################

add_subdirectory(lib/pipelib)    # Pipeline library
add_subdirectory(lib/sslib)      # Structure searching library


## Now StructurePipe ####################################

set(SPIPE_INCLUDE_DIRS
  ${PROJECT_BINARY_DIR}/src  # So that we find StructurePipeConfig.h
  ${PROJECT_SOURCE_DIR}/src
  CACHE FILEPATH INTERNAL FORCE
)

# Include directories with header files
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${PIPELIB_INCLUDE_DIRS}
  ${SSLIB_INCLUDE_DIRS}
  ${SPGLIB_INCLUDE_DIRS}
  ${SPIPE_INCLUDE_DIRS}
)

include_directories(
  lib/pipelib/include
  lib/sslib/include
)

add_subdirectory(src ${CMAKE_BINARY_DIR}/spipe)

set_property(TARGET PROPERTY PROJECT_LABEL "StructurePipe")


#
# Tests
if(SP_ENABLE_TESTING)
  add_subdirectory(tests)
endif(SP_ENABLE_TESTING)
