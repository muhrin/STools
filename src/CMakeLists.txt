
## stools

set(stools_Header_Files__
  STools.h
  ${PROJECT_BINARY_DIR}/src/SToolsConfig.h
)
source_group("Header Files" FILES ${stools_Header_Files__})

## stools/utility
set(stools_Header_Files__utility
  utility/TerminalFunctions.h
)
source_group("Header Files\\utility" FILES ${stools_Header_Files__utility})

set(stools_Source_Files__utility
  utility/TerminalFunctions.cpp
)
source_group("Source Files\\utility" FILES ${stools_Source_Files__utility})

set(stools_Files
  ${stools_Header_Files__}
  ${stools_Header_Files__utility}
  ${stools_Source_Files__utility}
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
  ${stools_Files}
  ${stools_Header_Files__sdiff}
  ${stools_Source_Files__sdiff}
)

#######################################
## sgroup: Get structure space group ##
#######################################
set(stools_Header_Files__sgroup
)
source_group("Header Files" FILES ${stools_Header_Files__sgroup})

set(stools_Source_Files__sgroup
  sgroup.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__sgroup})

set(stools_Files__sgroup
  ${stools_Header_Files__sgroup}
  ${stools_Source_Files__sgroup}
)

add_executable(sgroup
  ${stools_Files}
  ${stools_Files__sgroup}
)

add_dependencies(sgroup sslib)

# Libraries we need to link to
target_link_libraries(sgroup
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${ARMADILLO_LIBRARIES}
  sslib
)

## sinfo
set(stools_Header_Files__sinfo
)
source_group("Header Files" FILES ${stools_Header_Files__sinfo})

set(stools_Source_Files__sinfo
  sinfo.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__sinfo})

set(stools_Files__sinfo
  ${stools_Files}
  ${stools_Header_Files__sinfo}
  ${stools_Source_Files__sinfo}
)

## ssearch
set(stools_Header_Files__ssearch
)
source_group("Header Files" FILES ${stools_Header_Files__ssearch})

set(stools_Source_Files__ssearch
  ssearch.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__ssearch})

set(stools_Files__ssearch
  ${stools_Files}
  ${stools_Header_Files__ssearch}
  ${stools_Source_Files__ssearch}
)

## ssweep

set(stools_Header_Files__ssweep
)
source_group("Header Files" FILES ${stools_Header_Files__ssweep})

set(stools_Source_Files__ssweep
  ssweep.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__ssweep})

set(stools_Files__ssweep
  ${stools_Files}
  ${stools_Header_Files__ssweep}
  ${stools_Source_Files__ssweep}
)


#############################################
## sdiff: Structural difference calculator ##
#############################################
add_executable(sdiff
  ${stools_Files}
  ${stools_Files__sdiff}
)

add_dependencies(sdiff spipe)

# Libraries we need to link to
target_link_libraries(sdiff
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${ARMADILLO_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${SPGLIB_LIBRARIES}
  spipe
)

########################################
## sinfo: Print info about structures ##
########################################
add_executable(sinfo
  ${stools_Files}
  ${stools_Files__sinfo}
)

add_dependencies(sinfo sslib)

# Libraries we need to link to
target_link_libraries(sinfo
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${ARMADILLO_LIBRARIES}
  sslib
)

#######################################
## ssearch: General structure search ##
#######################################
add_executable(ssearch
  ${stools_Files}
  ${stools_Files__ssearch}
)

add_dependencies(ssearch spipe)

# Libraries we need to link to
target_link_libraries(ssearch
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${ARMADILLO_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  spipe
)

###############################################
## ssweep: Potential and stoichiometry sweep ##
###############################################
add_executable(ssweep
  ${stools_Files}
  ${stools_Files__ssweep}
)

add_dependencies(ssweep spipe)

# Libraries we need to link to
target_link_libraries(ssweep
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}	# filesystem depends on this
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${ARMADILLO_LIBRARIES}
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
  
  
