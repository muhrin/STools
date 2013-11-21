/*
 * MapEntries.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SPIPE__FACTORY__MAP_ENTRIES_H
#define SPIPE__FACTORY__MAP_ENTRIES_H

// INCLUDES /////////////////////////////////////////////
#include <string>
#include <vector>

#include <spl/build_cell/AtomsDescription.h>
#include <spl/utility/HeterogeneousMapKey.h>

// DEFINES //////////////////////////////////////////////

namespace spipe {
namespace factory {

///////////////////////////////////////////////////////////
// MAP KEYS
///////////////////////////////////////////////////////////

// GENERAL
extern ::spl::utility::Key< int> NUM;
extern ::spl::utility::Key< double> PERCENT;
extern ::spl::utility::Key< ::std::string> RNG_SEED;

// BLOCKS
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> BUILD_STRUCTURES;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> FIND_SYMMETRY_GROUP;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> KEEP_TOP_N;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> KEEP_WITHIN_X_PERCENT;
extern ::spl::utility::Key< ::std::string> LOAD_STRUCTURES;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> NIGGLI_REDUCE;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> GEOM_OPTIMISE;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> REMOVE_DUPLICATES;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> RUN_POTENTIAL_PARAMS_QUEUE;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> SEARCH_STOICHIOMETRIES;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> SWEEP_POTENTIAL_PARAMS;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> WRITE_STRUCTURES;

extern ::spl::utility::Key<
    ::std::map< ::std::string, ::spl::build_cell::AtomsDescription::CountRange> > ATOM_RANGES;

extern ::spl::utility::Key< ::std::vector< ::std::string> > PARAM_RANGE;
extern ::spl::utility::Key< bool> MULTI_WRITE;
extern ::spl::utility::Key< ::std::string> FORMAT;

extern ::spl::utility::Key< ::std::string> QUEUE_FILE;
extern ::spl::utility::Key< ::std::string> DONE_FILE;

extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> PRE_GEOM_OPTIMISE;

}
}

#endif /* SPIPE__FACTORY__MAP_ENTRIES_H */

