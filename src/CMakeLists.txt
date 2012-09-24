
## stools

set(stools_Header_Files__
  STools.h
  SToolsConfig.h
)
source_group("Header Files" FILES ${stools_Header_Files__})

set(stools_Files
  ${stools_Header_Files__}
)

## sdiff
set(stools_Header_Files__sdiff
)
source_group("Header Files" FILES ${stools_Header_Files__sdiff})

set(stools_Source_Files__sdiff
  sdiff.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__sdiff})

set(stools_Files__sdiff
  ${stools_Header_Files__sdif}
  ${stools_Source_Files__sdiff}
)

## ssweep

set(stools_Header_Files__ssweep
)
source_group("Header Files\\ssweep" FILES ${stools_Header_Files__ssweep})

set(stools_Source_Files__ssweep
  ssweep/ssweep.cpp
)
source_group("Source Files\\ssweep" FILES ${stools_Source_Files__ssweep})

set(stools_Files__ssweep
  ${stools_Header_Files__ssweep}
  ${stools_Source_Files__ssweep}
)

#########################
## Include directories ##
#########################

include_directories(
  ${CMAKE_SOURCE_DIR}/lib/yaml-cpp/include
  ${CMAKE_SOURCE_DIR}/lib/pipelib/include
  ${CMAKE_SOURCE_DIR}/lib/sslib/include
  ${CMAKE_SOURCE_DIR}/lib/sslib/src # TODO: Remove this and move sslib/src/SSLib* to include
  ${CMAKE_SOURCE_DIR}/lib/spglib/include
  ${CMAKE_SOURCE_DIR}/lib/spipe/src
)

include_directories(.)


#######################################
## Potential and stoichiometry sweep ##
#######################################
add_executable(ssweep
  ${stools_Files}
  ${stools_Files__ssweep}
)

add_dependencies(ssweep yaml_cpp spglib sslib pipelib spipe)

# Libraries we need to link to
target_link_libraries(ssweep
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
  ${ARMADILLO_LIBRARIES}
  yaml-cpp
  spglib
  sslib
  pipelib
  spipe
)


##################
## StructureMap ##
##################
#add_executable(smap
#  ${spipe_Files}
#  MakeMap.cpp
#)

#add_dependencies(smap sslib)

# Libraries we need to link to
#  target_link_libraries(smap
#    ${Boost_FILESYSTEM_LIBRARY}
#    ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
#    ${ARMADILLO_LIBRARIES}
#    sslib
#  )
  

###################
## StructureDiff ##
###################
add_executable(sdiff
  ${stools_Files}
  ${stools_Files__sdiff}
)

add_dependencies(sdiff spglib sslib pipelib spipe)

# Libraries we need to link to
target_link_libraries(sdiff
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${ARMADILLO_LIBRARIES}
  yaml-cpp
  spglib
  sslib
  pipelib
  spipe
)
  
