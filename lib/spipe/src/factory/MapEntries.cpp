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
::spl::utility::Key<int> NUM;
::spl::utility::Key< ::std::string> RNG_SEED;

// BLOCKS
::spl::utility::Key< ::spl::utility::HeterogeneousMap> LOWEST_ENERGY;
::spl::utility::Key<size_t> KEEP_TOP;
::spl::utility::Key<double> KEEP_WITHIN;

::spl::utility::Key< ::spl::utility::HeterogeneousMap> GEOM_OPTIMISE;

::spl::utility::Key< ::spl::utility::HeterogeneousMap> PARAM_SWEEP;
::spl::utility::Key< ::std::vector< ::std::string> > PARAM_RANGE;

::spl::utility::Key< ::spl::utility::HeterogeneousMap> PRE_GEOM_OPTIMISE;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> RANDOM_STRUCTURE;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> REMOVE_DUPLICATES;

::spl::utility::Key< ::spl::utility::HeterogeneousMap> WRITE_STRUCTURES;
::spl::utility::Key<bool> MULTI_WRITE;
::spl::utility::Key< ::std::string> FILE_TYPE;

}
}


