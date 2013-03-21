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

// GENERAL
::sstbx::utility::Key<int> NUM;

// BLOCKS
::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> LOWEST_ENERGY;
::sstbx::utility::Key<size_t> KEEP_TOP;
::sstbx::utility::Key<double> KEEP_WITHIN;

::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> GEOM_OPTIMISE;

::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> PARAM_SWEEP;
::sstbx::utility::Key< ::std::vector< ::std::string> > PARAM_RANGE;

::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> PRE_GEOM_OPTIMISE;
::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> RANDOM_STRUCTURE;
::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> REMOVE_DUPLICATES;

::sstbx::utility::Key< ::sstbx::utility::HeterogeneousMap> WRITE_STRUCTURES;
::sstbx::utility::Key<bool> MULTI_WRITE;
::sstbx::utility::Key< ::std::string> FILE_TYPE;

}
}


