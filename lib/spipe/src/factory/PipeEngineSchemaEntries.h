/*
 * PipeEngineSchemaEntries.h
 *
 *
 *  Created on: Sep 10, 2013
 *      Author: Martin Uhrin
 */

#ifndef PIPE_ENGINE_SCHEMA_ENTRIES_H
#define PIPE_ENGINE_SCHEMA_ENTRIES_H

// INCLUDES /////////////////////////////////////////////

#include <spl/utility/HeterogeneousMapKey.h>

// DEFINES //////////////////////////////////////////////

namespace spipe {
namespace factory {

///////////////////////////////////////////////////////////
// MAP KEYS
///////////////////////////////////////////////////////////

// Engines
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> ENGINE;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> SERIAL;
extern ::spl::utility::Key< ::spl::utility::HeterogeneousMap> BOOST_THREAD;

extern ::spl::utility::Key< int> NUM_THREADS;

}
}

#endif /* PIPE_ENGINE_SCHEMA_ENTRIES_H */

