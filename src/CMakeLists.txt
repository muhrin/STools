
## stools

set(stools_Header_Files__
  STools.h
  ${PROJECT_BINARY_DIR}/src/SToolsConfig.h
)
source_group("Header Files" FILES ${stools_Header_Files__})

## stools/factory
set(stools_Header_Files__factory
  factory/PipeEngine.h
  factory/StFactory.h
  factory/YamlSchema.h
)
source_group("Header Files\\factory" FILES ${stools_Header_Files__factory})

set(stools_Source_Files__factory
  factory/StFactory.cpp
)
source_group("Source Files\\factory" FILES ${stools_Source_Files__factory})

## stools/input
set(stools_Header_Files__input
  input/OptionsParsing.h
  input/YamlKeywords.h
)
source_group("Header Files\\input" FILES ${stools_Header_Files__input})

set(stools_Source_Files__input
  input/OptionsParsing.cpp
)
source_group("Source Files\\input" FILES ${stools_Source_Files__input})

## stools/utility
set(stools_Header_Files__utility
  utility/BoostCapabilities.h
  utility/CustomTokens.h
  utility/InfoToken.h
  utility/StringParsing.h
  utility/TerminalFunctions.h
  utility/YamlOptionsParser.h
)
source_group("Header Files\\utility" FILES ${stools_Header_Files__utility})

set(stools_Source_Files__utility
  utility/CustomTokens.cpp
  utility/InfoToken.cpp
  utility/StringParsing.cpp
  utility/TerminalFunctions.cpp
)
source_group("Source Files\\utility" FILES ${stools_Source_Files__utility})

set(stools_Header_Files__utility__detail
  utility/detail/InfoToken.h
)
source_group("Header Files\\utility\\detail" FILES ${stools_Header_Files__utility__detail})

set(stools_Files
  ${stools_Header_Files__}
  ${stools_Header_Files__input}
  ${stools_Source_Files__input}
  ${stools_Header_Files__factory}
  ${stools_Source_Files__factory}
  ${stools_Header_Files__utility}
  ${stools_Source_Files__utility}
  ${stools_Header_Files__utility__detail}
)

add_library(stools_common
  ${stools_Files}
)

add_dependencies(stools_common sslib spipe)

# Libraries we need to link to
target_link_libraries(stools_common
  ${Boost_LIBRARIES}
  sslib
  spipe
)

#################################
## sbuild: Build a structure   ##
#################################

set(stools_Header_Files__sbuild
)
source_group("Header Files" FILES ${stools_Header_Files__sbuild})

set(stools_Source_Files__sbuild
  sbuild.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__sbuild})

set(stools_Files__sbuild
  ${stools_Header_Files__sbuild}
  ${stools_Source_Files__sbuild}
)

add_executable(sbuild
  ${stools_Files__sbuild}
)

add_dependencies(sbuild stools_common)

# Libraries we need to link to
target_link_libraries(sbuild
  stools_common
)

set(stools_targets
  ${stools_targets}
  sbuild
)

##############################################
## sconvert: Convert structure file formats ##
##############################################
set(stools_Header_Files__sconvert
)
source_group("Header Files" FILES ${stools_Header_Files__sconvert})

set(stools_Source_Files__sconvert
  sconvert.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__sconvert})

set(stools_Files__sconvert
  ${stools_Header_Files__sconvert}
  ${stools_Source_Files__sconvert}
)

add_executable(sconvert
  ${stools_Files__sconvert}
)

add_dependencies(sconvert stools_common)

# Libraries we need to link to
target_link_libraries(sconvert
  stools_common
)

set(stools_targets
  ${stools_targets}
  sconvert
)

#############################################
## sdiff: Structural difference calculator ##
#############################################
set(stools_Header_Files__sdiff
)
source_group("Header Files" FILES ${stools_Header_Files__sdiff})

set(stools_Source_Files__sdiff
  sdiff.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__sdiff})

set(stools_Files__sdiff
  ${stools_Header_Files__sdiff}
  ${stools_Source_Files__sdiff}
)

add_executable(sdiff
  ${stools_Files__sdiff}
)

add_dependencies(sdiff stools_common)

# Libraries we need to link to
target_link_libraries(sdiff
  stools_common
)

set(stools_targets
  ${stools_targets}
  sdiff
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
  ${stools_Files__sgroup}
)

add_dependencies(sgroup stools_common)

# Libraries we need to link to
target_link_libraries(sgroup
  stools_common
)

set(stools_targets
  ${stools_targets}
  sgroup
)

#############################################
## shull: Structures convex hull generator ##
#############################################
if(SSLIB_USE_CGAL)

set(stools_Header_Files__shull
)
source_group("Header Files" FILES ${stools_Header_Files__shull})

set(stools_Source_Files__shull
  shull.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__shull})

set(stools_Files__shull
  ${stools_Header_Files__shull}
  ${stools_Source_Files__shull}
)

add_executable(shull
  ${stools_Files__shull}
)

add_dependencies(shull stools_common)

# Libraries we need to link to
target_link_libraries(shull
  stools_common
)

set(stools_targets
  ${stools_targets}
  shull
)

endif(SSLIB_USE_CGAL)

########################################
## sinfo: Print info about structures ##
########################################

set(stools_Header_Files__sinfo
  sinfo/Ancillary.h
)
source_group("Header Files\\sinfo" FILES ${stools_Header_Files__sinfo})

set(stools_Source_Files__sinfo
  sinfo/Ancillary.cpp
)
source_group("Source Files\\sinfo" FILES ${stools_Source_Files__sinfo})

set(stools_Files__sinfo
  ${stools_Header_Files__sinfo}
  ${stools_Source_Files__sinfo}
  sinfo.cpp
)

add_executable(sinfo
  ${stools_Files__sinfo}
)

add_dependencies(sinfo stools_common)

# Libraries we need to link to
target_link_libraries(sinfo
  stools_common
)

set(stools_targets
  ${stools_targets}
  sinfo
)

########################
## slock: Lock a file ##
########################

set(stools_Header_Files__slock
)
source_group("Header Files\\slock" FILES ${stools_Header_Files__slock})

set(stools_Source_Files__slock
)
source_group("Source Files\\sinfo" FILES ${stools_Source_Files__slock})

set(stools_Files__slock
  ${stools_Header_Files__slock}
  ${stools_Source_Files__slock}
  slock.cpp
)

add_executable(slock
  ${stools_Files__slock}
)

find_package(Boost 1.36.0 COMPONENTS date_time filesystem system
  thread program_options REQUIRED)

# Libraries we need to link to
target_link_libraries(slock
  ${Boost_LIBRARIES}
)

set(stools_targets
  ${stools_targets}
  slock
)

############################################
## smeasure: Structural measurements tool ##
############################################
set(stools_Header_Files__smeasure
)
source_group("Header Files" FILES ${stools_Header_Files__smeasure})

set(stools_Source_Files__smeasure
  smeasure.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__smeasure})

set(stools_Files__smeasure
  ${stools_Header_Files__smeasure}
  ${stools_Source_Files__smeasure}
)

add_executable(smeasure
  ${stools_Files__smeasure}
)

add_dependencies(smeasure stools_common)

# Libraries we need to link to
target_link_libraries(smeasure
  stools_common
)

set(stools_targets
  ${stools_targets}
  smeasure
)

#######################################
## ssearch: General structure search ##
#######################################

set(stools_Header_Files__ssearch
)
source_group("Header Files" FILES ${stools_Header_Files__ssearch})

set(stools_Source_Files__ssearch
  ssearch.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__ssearch})

set(stools_Files__ssearch
  ${stools_Header_Files__ssearch}
  ${stools_Source_Files__ssearch}
)

add_executable(ssearch
  ${stools_Files__ssearch}
)

add_dependencies(ssearch stools_common)

# Libraries we need to link to
target_link_libraries(ssearch
  stools_common
)

set(stools_targets
  ${stools_targets}
  ssearch
)

############################################################
## strian: Structure triangulation for nearest neighbour, ##
## coordination polyhedra etc.                            ##
############################################################
if(SSLIB_USE_CGAL)

set(stools_Header_Files__strian
)
source_group("Header Files" FILES ${stools_Header_Files__ssearch})

set(stools_Source_Files__strian
  strian.cpp
)
source_group("Source Files" FILES ${stools_Source_Files__ssearch})

set(stools_Files__strian
  ${stools_Header_Files__strian}
  ${stools_Source_Files__strian}
)

add_executable(strian
  ${stools_Files__strian}
)

add_dependencies(strian stools_common)

# Libraries we need to link to
target_link_libraries(strian
  stools_common
)

set(stools_targets
  ${stools_targets}
  strian
)

endif(SSLIB_USE_CGAL)

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
#    ${Boost_LIBRARIES}
#    sslib
#  )
  
  
## Installation #####

INSTALL(TARGETS ${stools_targets}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
