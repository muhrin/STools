

cmake_minimum_required(VERSION 2.6)

## SCRIPTS TO BE INSTALLED #################################

# All bash scripts
if(UNIX)
  set(STOOLS_SCRIPTS
    scripts/castep_manip.inc
    scripts/castep_qsub_optimiser
    scripts/sgen_work
    scripts/slowest
    scripts/potparams2queue
    scripts/srefine
    scripts/srun
    scripts/qsub_remote
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

