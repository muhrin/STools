
## Configure the tests header file

# Figure out if we are using dynamic or static boost libraries
if(Boost_USE_STATIC_LIBS)
else()
  set(SSLIB_BOOST_DYN_LINK ON)
endif()

configure_file(
  "${PROJECT_SOURCE_DIR}/tests/sslibtest.h.in"
  "${PROJECT_BINARY_DIR}/tests/sslibtest.h"
)

# tests/build_cell

set(tests_Source_Files__build_cell
  build_cell/AtomExtruderTest.cpp
)
source_group("Source Files\\build_cell" FILES ${tests_Source_Files__build_cell})

# tests/common

set(tests_Source_Files__common
  common/DistanceCalculatorsTest.cpp
  common/UnitCellTest.cpp
)
source_group("Source Files\\common" FILES ${tests_Source_Files__common})

# tests/factory

set(tests_Source_Files__factory
  factory/YamlFactoryTest.cpp
)
source_group("Source Files\\factory" FILES ${tests_Source_Files__factory})

## tests/utility

set(tests_Header_Files__utility
)
source_group("Header Files\\utility" FILES ${tests_Header_Files__utility})

set(tests_Source_Files__utility
  utility/MultiIdxTest.cpp
  utility/MultiIdxRangeTest.cpp
  utility/PermutationRangeTest.cpp
)
source_group("Source Files\\utility" FILES ${tests_Source_Files__utility})

## tests/

set(tests_Header_Files__
  "${PROJECT_BINARY_DIR}/tests/sslibtest.h"
)
source_group("Header Files" FILES ${tests_Header_Files__})

set(tests_Source_Files__
)
source_group("Source Files" FILES ${tests_Source_Files__})

set(tests_Header_Files
  ${tests_Header_Files__utility}
  ${tests_Header_Files__}
)

set(tests_Source_Files
  ${tests_Source_Files__build_cell}
  ${tests_Source_Files__common}
  ${tests_Source_Files__factory}
  ${tests_Source_Files__utility}
  ${tests_Source_Files__}
)

set(tests_Files
  ${tests_Header_Files}
  ${tests_Source_Files}
)


#########################
## Include directories ##
#########################

include_directories(
  ${SSLIB_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}/tests # To make sure we can find sslibtest.h (generated from .in)
  ${PROJECT_SOURCE_DIR}/tests
)


############################
## SSLibTest executable   ##
############################
add_executable(sslibtest
  ${tests_Files}
  sslibtest.cpp
)

add_dependencies(sslibtest sslib)

# Libraries we need to link to
target_link_libraries(sslibtest
  ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
  ${ARMADILLO_LIBRARIES}
  spglib
  sslib
)

