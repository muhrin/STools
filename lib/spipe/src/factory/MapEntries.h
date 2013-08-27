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

#include "spl/utility/HeterogeneousMapKey.h"

// DEFINES //////////////////////////////////////////////

namespace spipe {
namespace factory {

///////////////////////////////////////////////////////////
// MAP KEYS
///////////////////////////////////////////////////////////

// GENERAL
extern ::spl::utility::Key<int> NUM;
extern ::spl::utility::Key< ::std::string> RNG_SEED;

// BLOCKS
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> LOWEST_ENERGY;
extern ::spl::utility::Key<size_t> KEEP_TOP;
extern ::spl::utility::Key<double> KEEP_WITHIN;

extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> GEOM_OPTIMISE;

extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> PARAM_SWEEP;
extern ::spl::utility::Key< ::std::vector< ::std::string> > PARAM_RANGE;

extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> PRE_GEOM_OPTIMISE;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> RANDOM_STRUCTURE;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> REMOVE_DUPLICATES;

extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> WRITE_STRUCTURES;
extern ::spl::utility::Key<bool> MULTI_WRITE;
extern ::spl::utility::Key< ::std::string> FILE_TYPE;




}
}

#endif /* SPIPE__FACTORY__MAP_ENTRIES_H */

