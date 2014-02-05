
## spipe/blocks/

set(spipe_Header_Files__blocks
  blocks/BuildStructures.h
  blocks/Clone.h
  blocks/CutAndPaste.h
  blocks/FindSymmetryGroup.h
  blocks/EdgeDetect.h
  blocks/GeomOptimise.h
  blocks/KeepStableCompositions.h
  blocks/KeepTopN.h
  blocks/KeepWithinXPercent.h
  blocks/LoadStructures.h
  blocks/NiggliReduce.h
  blocks/ParamGeomOptimise.h
  blocks/PasteFragment.h
  blocks/RemoveDuplicates.h
  blocks/RunPotentialParamsQueue.h
  blocks/SearchStoichiometries.h
  blocks/SeparateAtoms.h
  blocks/SweepPotentialParams.h
  blocks/WriteStructures.h
)
source_group("Header Files\\blocks" FILES ${spipe_Header_Files__blocks})

set(spipe_Source_Files__blocks
  blocks/BuildStructures.cpp
  blocks/Clone.cpp
  blocks/CutAndPaste.cpp
  blocks/FindSymmetryGroup.cpp
  blocks/GeomOptimise.cpp
  blocks/KeepStableCompositions.cpp
  blocks/KeepTopN.cpp
  blocks/KeepWithinXPercent.cpp
  blocks/LoadStructures.cpp
  blocks/NiggliReduce.cpp
  blocks/ParamGeomOptimise.cpp
  blocks/PasteFragment.cpp
  blocks/RemoveDuplicates.cpp
  blocks/RunPotentialParamsQueue.cpp
  blocks/SearchStoichiometries.cpp
  blocks/SeparateAtoms.cpp
  blocks/SweepPotentialParams.cpp
  blocks/WriteStructures.cpp
)
source_group("Source Files\\blocks" FILES ${spipe_Source_Files__blocks})

## spipe/build/

set(spipe_Header_Files__build
  build/BlockLoader.h
  build/PipeBuilder.h
)
source_group("Header Files\\build" FILES ${spipe_Header_Files__build})

set(spipe_Source_Files__build
  build/BlockLoader.cpp
  build/PipeBuilder.cpp
)
source_group("Source Files\\build" FILES ${spipe_Source_Files__build})

## spipe/common/

set(spipe_Header_Files__common
  common/Assert.h
  common/CommonData.h
  common/GlobalData.h
  common/PipeBuilder.h
  common/PipeFunctions.h
  common/SharedData.h
  common/StructureData.h
  common/UtilityFunctions.h
)
source_group("Header Files\\common" FILES ${spipe_Header_Files__common})

set(spipe_Source_Files__common
  common/CommonData.cpp
  common/GlobalData.cpp
  common/PipeBuilder.cpp
  common/PipeFunctions.cpp
  common/SharedData.cpp
  common/StructureData.cpp
  common/UtilityFunctions.cpp
)
source_group("Source Files\\common" FILES ${spipe_Source_Files__common})

## spipe/factory/

set(spipe_Header_Files__factory
  factory/BlockFactory.h
  factory/PipeEngineFactory.h
  factory/PipeEngineSchema.h
  factory/YamlSchema.h
)
source_group("Header Files\\factory" FILES ${spipe_Header_Files__factory})

set(spipe_Source_Files__factory
  factory/BlockFactory.cpp
  factory/PipeEngineFactory.cpp
)
source_group("Source Files\\factory" FILES ${spipe_Source_Files__factory})

## spipe/io/

set(spipe_Header_Files__io
  io/BlockLoaderType.h
)
source_group("Header Files\\io" FILES ${spipe_Header_Files__io})

set(spipe_Source_Files__io
  io/BlockLoaderType.cpp
)
source_group("Source Files\\io" FILES ${spipe_Source_Files__io})

## spipe/utility/

set(spipe_Header_Files__utility
  utility/PipeDataInitialisation.h
  utility/DataTable.h
  utility/DataTableInserters.h
  utility/DataTableSupport.h
  utility/DataTableWriter.h
  utility/IDataTableChangeListener.h
  utility/Initialisation.h
)
source_group("Header Files\\utility" FILES ${spipe_Header_Files__utility})

set(spipe_Source_Files__utility
  utility/PipeDataInitialisation.cpp
  utility/DataTable.cpp
  utility/DataTableInserters.cpp
  utility/DataTableSupport.cpp
  utility/DataTableWriter.cpp
  utility/Initialisation.cpp
)
source_group("Source Files\\utility" FILES ${spipe_Source_Files__utility})

## spipe/

set(spipe_Header_Files__
  PipeLibTypes.h
  SpTypes.h
  StructurePipe.h
  StructurePipeConfig.h
)
source_group("Header Files" FILES ${spipe_Header_Files__})

set(spipe_Source_Files__
)
source_group("Source Files" FILES ${spipe_Source_Files__})

set(spipe_Header_Files
  ${spipe_Header_Files__blocks}
  ${spipe_Header_Files__build}
  ${spipe_Header_Files__common}
  ${spipe_Header_Files__factory}
  ${spipe_Header_Files__io}
  ${spipe_Header_Files__utility}
  ${spipe_Header_Files__}
)

set(spipe_Source_Files
  ${spipe_Source_Files__blocks}
  ${spipe_Source_Files__build}
  ${spipe_Source_Files__common}
  ${spipe_Source_Files__factory}
  ${spipe_Source_Files__io}
  ${spipe_Source_Files__utility}
  ${spipe_Source_Files__}
)

set(spipe_Files
  ${spipe_Header_Files}
  ${spipe_Source_Files}
)


#########################
## Include directories ##
#########################
include_directories(
  ${YAML_CPP_INCLUDE_DIRS}
  ${SPGLIB_INCLUDE_DIRS}
)

###########################
## StructurePipe Library ##
###########################
add_library(spipe
  ${spipe_Files}
)

# Libraries we need to link to
target_link_libraries(spipe
  ${Boost_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${SPGLIB_LIBRARIES}
  pipelib
  sslib
)

## Installation
install(TARGETS spipe
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

###################
## StructurePipe ##
###################
## Boost ##
# Disable auto-linking
add_definitions(-DBOOST_ALL_NO_LIB)
find_package(Boost 1.36.0 QUIET REQUIRED COMPONENTS system program_options)

# Make sure that after make install shared libraries are correctly linked
# to.  See http://www.cmake.org/Wiki/CMake_RPATH_handling#Common_questions
# for more details
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(spipe-bin
  spipe.cpp
)

add_dependencies(spipe-bin spipe)

set_target_properties(spipe-bin
  PROPERTIES OUTPUT_NAME spipe)

# Libraries we need to link to
target_link_libraries(spipe-bin
  ${Boost_LIBRARIES}
  spipe
)

## Installation
install(TARGETS spipe-bin
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
