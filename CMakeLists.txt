

cmake_minimum_required(VERSION 2.6)

## Include subprojects ################################

add_subdirectory(lib/spipe)	# StructurePipe

# Begin STools ########################################

project(STools)
set_property(TARGET PROPERTY PROJECT_LABEL "STools")

set(STOOLS_VERSION_MAJOR 0)
set(STOOLS_VERSION_MINOR 4)
set(STOOLS_VERSION_PATCH 0)
set(STOOLS_VERSION "${STOOLS_VERSION_MAJOR}.${STOOLS_VERSION_MINOR}.${STOOLS_VERSION_PATCH}")

message(STATUS "Configuring STools ${STOOLS_VERSION}")

## Configuration options ###############################

# Custom paths to libraries #########
set(ARMADILLO_ROOT "" CACHE FILEPATH "Path to armadillo linear algebra library")

## End configuration options ###########################

# Tell CMake where to find modules
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/;${CMAKE_MODULE_PATH}")

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
find_package(Armadillo QUIET NO_MODULE PATH_SUFFIXES share/Armadillo/CMake)
if(NOT Armadillo_FOUND)
  # Try finding using FindArmadillo
  find_package(Armadillo)
  # Unfortunately FindArmadillo set ARMADILLO_FOUND (caps) so set lower case manually now
  set(Armadillo_FOUND ${ARMADILLO_FOUND})
endif()

if(NOT Armadillo_FOUND)
  message(FATAL_ERROR "Armadillo not found, specify location via CMAKE_PREFIX_PATH or otherwise")
endif()
if("${ARMADILLO_INCLUDE_DIRS}" STREQUAL "")
  set(ARMADILLO_INCLUDE_DIRS "${ARMADILLO_ROOT}/include")
endif()

#
# Yaml
if(NOT YAML_CPP_LIBRARIES)
  find_package(YamlCpp REQUIRED)
endif(NOT YAML_CPP_LIBRARIES)

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
  ${SPIPE_INCLUDE_DIRS}
  ${SSLIB_INCLUDE_DIRS}
  ${STOOLS_INCLUDE_DIRS}
)

# Make sure that after make install shared libraries are correctly linked
# to.  See http://www.cmake.org/Wiki/CMake_RPATH_handling#Common_questions
# for more details
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(src ${CMAKE_BINARY_DIR}/bin)

## SCRIPTS TO BE INSTALLED #################################
if(UNIX)
  set(STOOLS_SCRIPTS
    scripts/castep_manip.inc
    scripts/castep_qsub_optimiser
    scripts/sgen_work
    scripts/potparams2queue
    scripts/srefine
    scripts/srun
    scripts/qsub_remote
  )
  install(PROGRAMS ${STOOLS_SCRIPTS}
    DESTINATION "bin" COMPONENT scripts
  )
  add_custom_target(install_scripts
    ALL COMMAND
    ${CMAKE_COMMAND}
    -DCOMPONENT=scripts
    -P cmake_install.cmake 
    DEPENDS ${STOOLS_SCRIPTS}
  )
endif(UNIX)

