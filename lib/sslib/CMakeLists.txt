cmake_minimum_required(VERSION 2.6)

project(SSLib)

# Tell CMake where to find modules
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/;${CMAKE_MODULE_PATH}")

set(SSLIB_VERSION_MAJOR 0)
set(SSLIB_VERSION_MINOR 2)
set(SSLIB_VERSION_PATCH 3)
set(SSLIB_VERSION_STRING "${SSLIB_VERSION_MAJOR}.${SSLIB_VERSION_MINOR}.${SSLIB_VERSION_PATCH}")

## CONFIGURATION SETTINGS ##############################

# Custom paths to libraries ####
set(BOOST_ROOT "" CACHE FILEPATH "Path to boost library")
set(ARMADILLO_ROOT "" CACHE FILEPATH "Path to armadillo linear algebra library")

# Build options ###
set(SSLIB_ENABLE_TESTING FALSE CACHE BOOL "Build sslib tests")
set(SSLIB_USE_YAML TRUE CACHE BOOL "SSLib should use YAML for input")

## END CONFIGURATION SETTINGS ##########################

message(STATUS "Configuring SSLib ${SSLIB_VERSION_STRING}")

configure_file(
  "${PROJECT_SOURCE_DIR}/include/SSLibConfig.h.in"
  "${PROJECT_BINARY_DIR}/include/SSLibConfig.h"
)


## FIND PACKAGES ########################################
# First add any user specified paths to the package search array
set(CMAKE_PREFIX_PATH "${BOOST_ROOT}" "${ARMADILLO_ROOT}")

# Boost #
if(WIN32)
  set(Boost_USE_STATIC_LIBS ON)
endif(WIN32)

# LAPACK (Required for parts of Armaillo library to work)
#find_package(LAPACK REQUIRED)

if(SSLIB_ENABLE_TESTING)
  find_package(Boost 1.36.0 COMPONENTS system filesystem unit_test_framework REQUIRED)
else()
  find_package(Boost 1.36.0 COMPONENTS system filesystem REQUIRED)
endif(SSLIB_ENABLE_TESTING)

# Armadillo #
find_package(Armadillo)
if("${ARMADILLO_INCLUDE_DIRS}" STREQUAL "")
  set(ARMADILLO_INCLUDE_DIRS "${ARMADILLO_ROOT}/include")
endif()

## END FIND PACKAGES ####################################

## INCLUDE DIRECTORIES ##################################

set(SSLIB_INCLUDE_DIRS
  ${PROJECT_BINARY_DIR}/include  # So that we find SSLibConfig.h
  ${PROJECT_SOURCE_DIR}/include
  CACHE FILEPATH INTERNAL FORCE
)

# Include directories with header files
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${SSLIB_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/lib/spglib/include     # TODO: Change this dependency
  ${CMAKE_SOURCE_DIR}/lib/yaml-cpp/include   # TODO: Change this dependency
)

# Include YAML if we're using it
if(SSLIB_USE_YAML)
  include_directories(${YAML_INCLUDE_DIRECTORIES})
endif(SSLIB_USE_YAML)

# Process header and source files
include(IncludeLists.txt)
include(SrcLists.txt)

set_property(TARGET PROPERTY PROJECT_LABEL "SSLib")

## BUILD SSLIB #############################################

add_library(sslib
  ${sslib_Header_Files}
  ${sslib_Source_Files}
)

# Libraries we need to link to
target_link_libraries(sslib
#  ${LAPACK_LIBRARIES}           # Required as we're using Armadillo
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
  spglib                        # TODO: Change this dependency
  yaml-cpp
)

if(SSLIB_USE_YAML)
  target_link_libraries(sslib
    ${YAML_LIBRARIES}
  )
endif(SSLIB_USE_YAML)


###########
## Tests ##
###########

if(SSLIB_ENABLE_TESTING)
  add_subdirectory(tests)
endif(SSLIB_ENABLE_TESTING)
