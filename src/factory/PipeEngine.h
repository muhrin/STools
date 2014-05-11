/*
 * PipeEngine.h
 *
 *
 *  Created on: Sep 10, 2013
 *      Author: Martin Uhrin
 */

#ifndef STOOLS_PIPE_ENGINE_H
#define STOOLS_PIPE_ENGINE_H

// INCLUDES /////////////////////////////////////////////
#include "STools.h"

#include <spl/utility/HeterogeneousMap.h>

#include <spipe/factory/factory.h>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace stools {
namespace factory {

typedef spipe::factory::PipeEngineFactory::EnginePtr PipeEnginePtr;

template <class T>
PipeEnginePtr
createPipeEngine(const T & options)
{
  static const spipe::factory::PipeEngineFactory ENGINE_FACTORY =
    spipe::factory::PipeEngineFactory();
  return ENGINE_FACTORY.createEngine(options.engine);
}

}
}

#endif /* STOOLS_PIPE_ENGINE_H */

