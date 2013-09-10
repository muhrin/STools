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

#include <spl/utility/HeterogeneousMap.h>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace factory {

class PipeEngineFactory
{
public:
  typedef ::spl::utility::HeterogeneousMap HeteroMap;
  typedef typename UniquePtr<Engine>::Type EnginePtr;

  EnginePtr
  createEngine(const HeteroMap & settings) const;
  UniquePtr<SerialEngine>::Type
  createSerialEngine(const HeteroMap & settings) const;
  UniquePtr<BoostThreadEngine>::Type
  createBoostThreadEngine(const HeteroMap & settings) const;
};

}
}

#endif /* PIPE_ENGINE_FACTORY_H */

