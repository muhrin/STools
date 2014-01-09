/*
 * PipeEngineFactory.h
 *
 *
 *  Created on: Sep 10, 2013
 *      Author: Martin Uhrin
 */

#ifndef PIPE_ENGINE_FACTORY_H
#define PIPE_ENGINE_FACTORY_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include "SpTypes.h"
#include "factory/PipeEngineSchema.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace factory {

class PipeEngineFactory
{
public:
  typedef UniquePtr< ::spipe::Engine>::Type EnginePtr;

  EnginePtr
  createEngine(const Engine & settings) const;
  UniquePtr< ::spipe::SerialEngine>::Type
  createSerialEngine(const SerialEngine & settings) const;
#ifdef SP_ENABLE_THREAD_AWARE
  UniquePtr< ::spipe::BoostThreadEngine>::Type
  createBoostThreadEngine(const BoostThreadEngine & settings) const;
#endif
};

}
}

#endif /* PIPE_ENGINE_FACTORY_H */

