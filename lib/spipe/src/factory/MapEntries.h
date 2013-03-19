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

#include "utility/HeterogeneousMapKey.h"

// DEFINES //////////////////////////////////////////////

namespace spipe {
namespace factory {

///////////////////////////////////////////////////////////
// MAP KEYS
///////////////////////////////////////////////////////////

// GENERAL
extern ::sstbx::utility::Key<int> NUM;

// BLOCKS
extern ::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> LOWEST_ENERGY;
extern ::sstbx::utility::Key<unsigned int> KEEP_TOP;
extern ::sstbx::utility::Key<double> KEEP_WITHIN;

extern ::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> GEOM_OPTIMISE;

extern ::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> PARAM_SWEEP;
extern ::sstbx::utility::Key< ::std::vector< ::std::string> > PARAM_RANGE;

extern ::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> PRE_GEOM_OPTIMISE;
extern ::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> RANDOM_STRUCTURE;
extern ::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> REMOVE_DUPLICATES;

extern ::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> WRITE_STRUCTURES;
extern ::sstbx::utility::Key<bool> MULTI_WRITE;
extern ::sstbx::utility::Key< ::std::string> FILE_TYPE;




}
}

#endif /* SPIPE__FACTORY__MAP_ENTRIES_H */

