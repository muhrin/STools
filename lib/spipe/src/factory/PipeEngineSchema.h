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

namespace spipe {
namespace factory {


struct SerialEngine
{
};

SCHEMER_MAP(SerialEngineSchema, SerialEngine)
{
}

struct BoostThreadEngine
{
  int nThreads;
};

SCHEMER_MAP(BoostThreadEngineSchema, BoostThreadEngine)
{
  element("nThreads", &BoostThreadEngine::nThreads);
}

struct Engine
{
  ::boost::optional< SerialEngine> serialEngine;
  ::boost::optional< BoostThreadEngine> boostThreadEngine;
};

SCHEMER_MAP(EngineSchema, Engine)
{
  element("serial", &Engine::serialEngine);
  element("multithreaded", &Engine::boostThreadEngine);
}

}
}

#endif /* PIPE_ENGINE_SCHEMA_H */
