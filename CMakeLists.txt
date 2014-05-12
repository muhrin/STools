

cmake_minimum_required(VERSION 2.6)

## External dependencies  #############################

# Begin STools ########################################

project(STools)
set_property(TARGET PROPERTY PROJECT_LABEL ${PROJECT_NAME})

set(STOOLS_VERSION_MAJOR 0)
set(STOOLS_VERSION_MINOR 4)
set(STOOLS_VERSION_PATCH 0)
set(STOOLS_VERSION "${STOOLS_VERSION_MAJOR}.${STOOLS_VERSION_MINOR}.${STOOLS_VERSION_PATCH}")

message(STATUS "Configuring ${PROJECT_NAME} ${STOOLS_VERSION}")

include(cmake/utility.cmake)

## Configuration options ###############################

configure_file(
  "${PROJECT_SOURCE_DIR}/src/SToolsConfig.h.in"
  "${PROJECT_BINARY_DIR}/src/SToolsConfig.h"
)

## FIND PACKAGES ########################################
# First add any user specified paths to the package search array
set(CMAKE_PREFIX_PATH "${ARMADILLO_ROOT}")

# Boost #
# Tell Windows builds to look for static boost libraries
if(WIN32)
  set(Boost_USE_STATIC_LIBS ON)
endif(WIN32)
find_package(Boost 1.36.0 COMPONENTS system filesystem program_options REQUIRED)

#
# Armadillo #
find_package(Armadillo QUIET CONFIG PATH_SUFFIXES share/Armadillo/CMake
REQUIRED)
if(NOT Armadillo_FOUND)
  # Try finding using FindArmadillo
  find_package(Armadillo)
  # Unfortunately FindArmadillo set ARMADILLO_FOUND (caps) so set lower case manually now
  set(Armadillo_FOUND ${ARMADILLO_FOUND})
endif()

if("${ARMADILLO_INCLUDE_DIRS}" STREQUAL "")
  set(ARMADILLO_INCLUDE_DIRS "${ARMADILLO_ROOT}/include")
endif()

find_package(YamlCpp REQUIRED)
find_package_or_install(Schemer)
find_package_or_install(SPL)
find_package_or_install(SPipe)
find_package(Spglib REQUIRED)

## END FIND PACKAGES ####################################

set(STOOLS_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/src # To find STools.h
  ${PROJECT_BINARY_DIR}/src # To find SToolsConfig.h
)

# Include directories with header files
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${PIPELIB_INCLUDE_DIRS}
  ${SCHEMER_INCLUDE_DIRS}
  ${SPL_INCLUDE_DIRS}
  ${SPIPE_INCLUDE_DIRS}
  ${STOOLS_INCLUDE_DIRS}
)

# Make sure that after make install shared libraries are correctly linked
# to.  See http://www.cmake.org/Wiki/CMake_RPATH_handling#Common_questions
# for more details
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(src ${CMAKE_BINARY_DIR}/bin)

## SCRIPTS #################################
add_subdirectory(scripts)

