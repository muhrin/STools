
## spipe/blocks/

set(spipe_Header_Files__blocks
  blocks/DetermineSpaceGroup.h
  blocks/EdgeDetect.h
  blocks/LoadPotStructures.h
  blocks/LowestFreeEnergy.h
  blocks/MakeConvexHull.h
  blocks/MakeMap.h
  blocks/NiggliReduction.h
  blocks/ParamPotentialGo.h
  blocks/PotentialGo.h
  blocks/PotentialParamSweep.h
  blocks/RandomStructure.h
  blocks/RemoveDuplicates.h
  blocks/StoichiometrySearch.h
  blocks/WriteStructure.h
)
source_group("Header Files\\blocks" FILES ${spipe_Header_Files__blocks})

set(spipe_Source_Files__blocks
  blocks/DetermineSpaceGroup.cpp
  blocks/LoadPotStructures.cpp
  blocks/LowestFreeEnergy.cpp
  blocks/MakeConvexHull.cpp
  blocks/MakeMap.cpp
  blocks/NiggliReduction.cpp
  blocks/ParamPotentialGo.cpp
  blocks/PotentialGo.cpp
  blocks/PotentialParamSweep.cpp
  blocks/RandomStructure.cpp
  blocks/RemoveDuplicates.cpp
  blocks/StoichiometrySearch.cpp
  blocks/WriteStructure.cpp
)
source_group("Source Files\\blocks" FILES ${spipe_Source_Files__blocks})

## spipe/common/

set(spipe_Header_Files__common
  common/PipeBuilder.h
  common/PipeFactoryYaml.h
  common/PipeFunctions.h
  common/SharedData.h
  common/SsLibFactoryYaml.h
  common/StructureData.h
  common/UtilityFunctions.h
  common/YamlInputObjectAdapter.h
  common/YamlInputObjectIteratorAdapter.h
)
source_group("Header Files\\common" FILES ${spipe_Header_Files__common})

set(spipe_Source_Files__common
  common/PipeBuilder.cpp
  common/PipeFactoryYaml.cpp
  common/PipeFunctions.cpp
  common/SharedData.cpp
  common/SsLibFactoryYaml.cpp
  common/StructureData.cpp
  common/UtilityFunctions.cpp
  common/YamlInputObjectAdapter.cpp
  common/YamlInputObjectIteratorAdapter.cpp
)
source_group("Source Files\\common" FILES ${spipe_Source_Files__common})

## spipe/utility/

set(spipe_Header_Files__utility
  utility/DataTable.h
  utility/DataTableSupport.h
  utility/DataTableValueChanged.h
  utility/DataTableWriter.h
  utility/IDataTableChangeListener.h
)
source_group("Header Files\\utility" FILES ${spipe_Header_Files__utility})

set(spipe_Source_Files__utility
  utility/DataTable.cpp
  utility/DataTableSupport.cpp
  utility/DataTableValueChanged.cpp
  utility/DataTableWriter.cpp
)
source_group("Source Files\\utility" FILES ${spipe_Source_Files__utility})

## spipe/

set(spipe_Header_Files__
  StructurePipe.h
  StructurePipeConfig.h
)
source_group("Header Files" FILES ${spipe_Header_Files__})

set(spipe_Source_Files__
)
source_group("Source Files" FILES ${spipe_Source_Files__})

set(spipe_Header_Files
  ${spipe_Header_Files__blocks}
  ${spipe_Header_Files__common}
  ${spipe_Header_Files__utility}
  ${spipe_Header_Files__}
)

set(spipe_Source_Files
  ${spipe_Source_Files__blocks}
  ${spipe_Source_Files__common}
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
  ${CMAKE_SOURCE_DIR}/lib/pipelib/include
  ${CMAKE_SOURCE_DIR}/lib/sslib/include
  ${SSLIB_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/lib/yaml-cpp/include
  ${CMAKE_SOURCE_DIR}/lib/spglib/include
  ${SPIPE_INCLUDE_DIRS}
)

###########################
## StructurePipe Library ##
###########################
add_library(spipe
  ${spipe_Files}
)

# Libraries we need to link to
target_link_libraries(spipe
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
  ${ARMADILLO_LIBRARIES}
  yaml-cpp
  spglib
  pipelib
  sslib
)

# Qhull
if(SP_USE_QHULL)
  target_link_libraries(spipe
    ${QHULL_LIBRARIES}
  )
endif(SP_USE_QHULL)

###################
## StructurePipe ##
###################
add_executable(spipe-bin
  StructurePipe.cpp
)

# Force the spipe-bin executable to be called just spipe
#set_target_properties(spipe-bin
#  PROPERTIES OUTPUT_NAME spipe)

add_dependencies(spipe-bin spipe)

# Libraries we need to link to
target_link_libraries(spipe-bin
  spipe
)

