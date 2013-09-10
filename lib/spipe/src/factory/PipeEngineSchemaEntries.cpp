/*
 * PipeEngineSchemaEntries.cpp
 *
 *  Created on: Sep 10, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "factory/PipeEngineSchemaEntries.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace factory {

// Engines
::spl::utility::Key< ::spl::utility::HeterogeneousMap> ENGINE;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> SERIAL;
::spl::utility::Key< ::spl::utility::HeterogeneousMap> BOOST_THREAD;

::spl::utility::Key< int> NUM_THREADS;

}
}


