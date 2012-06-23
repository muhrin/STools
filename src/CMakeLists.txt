
## stools

set(stools_Header_Files__
  STools.h
  SToolsConfig.h
)
source_group("Header Files" FILES ${stools_Header_Files__})

set(stools_Files
  ${stools_Header_Files__}
)

## stoich

set(stools_Header_Files__stoich
)
source_group("Header Files\\stoich" FILES ${stools_Header_Files__stoich})

set(stools_Source_Files__stoich
  stoich/stoich.cpp
)
source_group("Source Files\\stoich" FILES ${stools_Source_Files__stoich})

set(stools_Files__stoich
  ${stools_Header_Files__stoich}
  ${stools_Source_Files__stoich}
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


##########################
## Stoichiometry search ##
##########################
add_executable(stoich
  ${stools_Files}
  ${stools_Files__stoich}
)

add_dependencies(stoich yaml_cpp sslib  pipelib StructurePipe)

# Libraries we need to link to
  target_link_libraries(stoich
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
    ${ARMADILLO_LIBRARIES}
    yaml-cpp
    sslib
    pipelib
    spipe
  )
