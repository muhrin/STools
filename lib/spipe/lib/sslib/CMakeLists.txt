cmake_minimum_required(VERSION 2.6)

project(SSLib)

# Tell CMake where to find modules
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/;${CMAKE_MODULE_PATH}")

set(SSLIB_VERSION_MAJOR 0)
set(SSLIB_VERSION_MINOR 3)
set(SSLIB_VERSION_PATCH 0)
set(SSLIB_VERSION "${SSLIB_VERSION_MAJOR}.${SSLIB_VERSION_MINOR}.${SSLIB_VERSION_PATCH}")

message(STATUS "Configuring SSLib ${SSLIB_VERSION}")

## FIND PACKAGES ########################################

# Tell CMake where to find modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/modules/")

# First add any user specified paths to the package search array
set(CMAKE_PREFIX_PATH
  "${BOOST_ROOT}"
  "${ARMADILLO_ROOT}"
  "${CGAL_ROOT}"
  "${LAPACK_ROOT}"
)

#
# Boost #
# Disable auto-linking
add_definitions(-DBOOST_ALL_NO_LIB)
if(SSLIB_ENABLE_THREAD_AWARE)
  find_package(Boost 1.36.0 REQUIRED COMPONENTS system filesystem regex thread)
else()
  find_package(Boost 1.36.0 REQUIRED COMPONENTS system filesystem regex)
endif(SSLIB_ENABLE_THREAD_AWARE)

#
# Armadillo #
if(NOT ARMADILLO_INCLUDE_DIRS)
  find_package(Armadillo CONFIG REQUIRED)
  if("${ARMADILLO_INCLUDE_DIRS}" STREQUAL "")
    set(ARMADILLO_INCLUDE_DIRS "${ARMADILLO_ROOT}/include")
  endif()
endif(NOT ARMADILLO_INCLUDE_DIRS)

#
# CGAL #
if(SSLIB_USE_CGAL)
  find_package(CGAL REQUIRED)
  include( ${CGAL_USE_FILE} )
endif(SSLIB_USE_CGAL)

#
# spglib, only look if it hasn't already been found
if(NOT SPGLIB_LIBRARIES)
  find_package(Spglib REQUIRED)
endif(NOT SPGLIB_LIBRARIES)

#
# YamlCpp
find_package(YamlCpp QUIET)

## END FIND PACKAGES ####################################

## CONFIGURATION SETTINGS ##############################

# Build options ###
set(SSLIB_ENABLE_TESTING FALSE CACHE BOOL "Build sslib tests")
set(SSLIB_ENABLE_THREAD_AWARE TRUE CACHE BOOL "Enable awareness of multithreaded environment (requires Boost thread)")

set(SPL_WITH_CGAL ${CGAL_FOUND} CACHE INTERNAL "Build SPL with CGAL support.")
set(SPL_WITH_YAML ${YamlCpp_FOUND} CACHE INTERNAL "Build SPL with Yaml support.")

configure_file(
  "${PROJECT_SOURCE_DIR}/include/spl/SSLibConfig.h.in"
  "${PROJECT_BINARY_DIR}/include/spl/SSLibConfig.h"
)

## END CONFIGURATION SETTINGS ##########################

## EXTERNAL DEPENDENCIES ###############################

if(SPL_WITH_YAML)
  add_subdirectory(external/schemer)
  include_directories(external/schemer/include)
endif(SPL_WITH_YAML)

## END EXTERNAL DEPENDENCIES ###########################

## INCLUDE DIRECTORIES ##################################

set(SSLIB_LOCAL_INCLUDE_DIRS
  ${PROJECT_BINARY_DIR}/include  # So that we find SSLibConfig.h
  ${PROJECT_SOURCE_DIR}/include
  CACHE INTERNAL "Internal variable used to set the local include directories used by SSLib"
  FORCE
)

# Include directories with header files
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${SSLIB_LOCAL_INCLUDE_DIRS}
  ${SPGLIB_INCLUDE_DIRS}
)

# Include YAML if we're using it
if(SSLIB_USE_YAML)
  include_directories(${YAML_CPP_INCLUDE_DIRS})
endif(SSLIB_USE_YAML)

# Include CGAL if we're using it
if(SSLIB_USE_CGAL)
  include_directories(
    "${CGAL_INCLUDE_DIRS}"
  )
endif(SSLIB_USE_CGAL)

get_property(SSLIB_INCLUDE_DIRS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(SSLIB_INCLUDE_DIRS
  ${SSLIB_INCLUDE_DIRS}
  CACHE INTERNAL "Internal variable used to store all the include directories used by SSLib (including external libraries)"
  FORCE
)

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
  ${Boost_LIBRARIES}
  ${SPGLIB_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)

if(SPL_WITH_YAML)
  target_link_libraries(sslib
    ${YAML_CPP_LIBRARIES}
    schemer
  )
endif(SPL_WITH_YAML)

if(SPL_WITH_CGAL)
  target_link_libraries(sslib
    ${CGAL_LIBRARY}
    ${CGAL_Core_LIBRARY}
  )
endif(SPL_WITH_CGAL)

###########
## Tests ##
###########

if(SSLIB_ENABLE_TESTING)
  add_subdirectory(tests)
endif(SSLIB_ENABLE_TESTING)

