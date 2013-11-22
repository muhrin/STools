/*
 * MapEntries.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "factory/MapEntries.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace factory {

///////////////////////////////////////////////////////////
// MAP KEYS
///////////////////////////////////////////////////////////

// GENERAL
::spl::utility::Key< int> NUM;
::spl::utility::Key< double> PERCENT;
::spl::utility::Key< ::std::string> RNG_SEED;

// BLOCKS
::spl::utility::Key< ::spl::utility::HeterogeneousMap> BUILD_STRUCTURES;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> FIND_SYMMETRY_GROUP;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> KEEP_STABLE_COMPOSITIONS;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> KEEP_TOP_N;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> KEEP_WITHIN_X_PERCENT;
::spl::utility::Key< ::std::string> LOAD_STRUCTURES;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> NIGGLI_REDUCE;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> GEOM_OPTIMISE;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> REMOVE_DUPLICATES;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> RUN_POTENTIAL_PARAMS_QUEUE;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> SEARCH_STOICHIOMETRIES;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> SWEEP_POTENTIAL_PARAMS;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> WRITE_STRUCTURES;

// Stoichiometry search stuff
::spl::utility::Key<
    ::std::map< ::std::string, ::spl::build_cell::AtomsDescription::CountRange> > ATOM_RANGES;
::spl::utility::Key< bool> WRITE_HULL;

::spl::utility::Key< ::std::vector< ::std::string> > PARAM_RANGE;
::spl::utility::Key< bool> MULTI_WRITE;
::spl::utility::Key< ::std::string> FORMAT;

::spl::utility::Key< ::std::string> QUEUE_FILE;
::spl::utility::Key< ::std::string> DONE_FILE;

::spl::utility::Key< ::spl::utility::HeterogeneousMap> PRE_GEOM_OPTIMISE;

}
}

