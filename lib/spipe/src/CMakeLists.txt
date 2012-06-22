
## spipe/blocks/

set(spipe_Header_Files__blocks
  blocks/EdgeDetect.h
  blocks/LoadPotStructures.h
  blocks/LowestFreeEnergy.h
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
  blocks/LoadPotStructures.cpp
  blocks/LowestFreeEnergy.cpp
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
  common/DataTable.h
  common/DataTableValueChanged.h
  common/DataTableWriter.h
  common/IDataTableChangeListener.h
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
  common/DataTable.cpp
  common/DataTableValueChanged.cpp
  common/DataTableWriter.cpp
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
  ${spipe_Header_Files__}
)

set(spipe_Source_Files
  ${spipe_Source_Files__blocks}
  ${spipe_Source_Files__common}
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
  ${CMAKE_SOURCE_DIR}/lib/sslib/src # TODO: Remove this and move sslib/src/SSLib* to include
  ${CMAKE_SOURCE_DIR}/lib/yaml-cpp/include
)


include_directories(.)

###################
## StructurePipe ##
###################
add_library(spipe
  ${spipe_Files}
  StructurePipe.cpp
)

add_dependencies(spipe sslib yaml_cpp)

# Libraries we need to link to
  target_link_libraries(spipe
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
    ${ARMADILLO_LIBRARIES}
    sslib
    yaml-cpp
  )
