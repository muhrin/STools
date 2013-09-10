/*
 * PipeEngineSchema.h
 *
 *
 *  Created on: Sep 10, 2013
 *      Author: Martin Uhrin
 */

#ifndef PIPE_ENGINE_SCHEMA_H
#define PIPE_ENGINE_SCHEMA_H

// INCLUDES /////////////////////////////////////////////

#include <spl/factory/SsLibYamlSchema.h>

#include "factory/PipeEngineSchemaEntries.h"

namespace spipe {
namespace factory {

///////////////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////////////
typedef ::spl::factory::HeteroMap HeteroMap;

struct SerialEngine : HeteroMap
{
  SerialEngine()
  {}
};

struct BoostThreadEngine : HeteroMap
{
  BoostThreadEngine()
  {
    addScalarEntry("nThreads", NUM_THREADS)->element()->defaultValue(1);
  }
};

struct Engine : HeteroMap
{
  Engine()
  {
    addEntry("serial", SERIAL, new SerialEngine());
    addEntry("multithreaded", BOOST_THREAD, new BoostThreadEngine());
  }
};

}
}

#endif /* PIPE_ENGINE_SCHEMA_H */
