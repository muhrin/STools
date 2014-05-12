

cmake_minimum_required(VERSION 2.6)

## SCRIPTS TO BE INSTALLED #################################

# All bash scripts
if(UNIX)
  set(STOOLS_SCRIPTS
    castep_manip.inc
    castep_qsub_optimiser
    sgen_work
    slowest
    potparams2queue
    srefine
    srun
    qsub_remote
  )
  install(PROGRAMS ${STOOLS_SCRIPTS}
    DESTINATION "bin" COMPONENT scripts
  )
  add_custom_target(install_scripts
    COMMAND
    ${CMAKE_COMMAND}
    -DCOMPONENT=scripts
    -P cmake_install.cmake 
    DEPENDS ${STOOLS_SCRIPTS}
  )
endif(UNIX)

