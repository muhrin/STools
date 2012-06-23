

set(spglib_Header_Files__
  include/spglib/cell.h
  include/spglib/debug.h
  include/spglib/hall_symbol.h
  include/spglib/kpoint.h
  include/spglib/lattice.h
  include/spglib/mathfunc.h
  include/spglib/pointgroup.h
  include/spglib/primitive.h
  include/spglib/refinement.h
  include/spglib/site_symmetry.h
  include/spglib/sitesym_database.h
  include/spglib/spacegroup.h
  include/spglib/spg_database.h
  include/spglib/spglib.h
  include/spglib/spin.h
  include/spglib/symmetry.h
)
source_group("Header Files\\" FILES ${spglib_Header_Files__})

set(spglib_Source_Files__
  src/cell.c
  src/debug.c
  src/hall_symbol.c
  src/kpoint.c
  src/lattice.c
  src/mathfunc.c
  src/pointgroup.c
  src/primitive.c
  src/refinement.c
  src/site_symmetry.c
  src/sitesym_database.c
  src/spacegroup.c
  src/spg_database.c
  src/spglib.c
  src/spglib_f.c
  src/spin.c
  src/symmetry.c
)
source_group("Source Files\\" FILES ${spglib_Source_Files__})

set(spglib_Files
  ${spglib_Header_Files__}
  ${spglib_Source_Files__}
)

include_directories(include/spglib)

# Need to add this as the c files in spglib are used to living in same
# folder as header files
add_library(spglib STATIC ${spglib_Files})

# Suppress spglib warnings
set_target_properties(spglib PROPERTIES COMPILE_FLAGS "-w")
           
