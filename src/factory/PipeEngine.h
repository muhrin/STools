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

#include <factory/PipeEngineFactory.h>
#include <factory/PipeEngineSchemaEntries.h>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace stools {
namespace factory {

typedef ::spipe::factory::PipeEngineFactory::EnginePtr PipeEnginePtr;

PipeEnginePtr
createPipeEngine(const ::spl::utility::HeterogeneousMap & options)
{
  typedef ::spl::utility::HeterogeneousMap OptionsMap;

  const OptionsMap * const engineSettings = options.find(
      ::spipe::factory::ENGINE);
  if(engineSettings)
  {
    ::spipe::factory::PipeEngineFactory engineFactory;
    return engineFactory.createEngine(*engineSettings);
  }

  return PipeEnginePtr();
}

}
}

#endif /* STOOLS_PIPE_ENGINE_H */

