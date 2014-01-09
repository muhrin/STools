/*
 * PipeEngineFactory.cpp
 *
 *  Created on: Sep 10, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "factory/PipeEngineFactory.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace factory {

PipeEngineFactory::EnginePtr
PipeEngineFactory::createEngine(const Engine & settings) const
{
  if(settings.serialEngine)
    return EnginePtr(createSerialEngine(*settings.serialEngine).release());
#ifdef PIPELIB_USE_BOOST_THREAD
  else if(settings.boostThreadEngine)
    return EnginePtr(
        createBoostThreadEngine(*settings.boostThreadEngine).release());
#endif

  return EnginePtr();
}

UniquePtr< ::spipe::SerialEngine>::Type
PipeEngineFactory::createSerialEngine(const SerialEngine & settings) const
{
  return UniquePtr< ::spipe::SerialEngine>::Type(new ::spipe::SerialEngine);
}

#ifdef PIPELIB_USE_BOOST_THREAD
UniquePtr< ::spipe::BoostThreadEngine>::Type
PipeEngineFactory::createBoostThreadEngine(
    const BoostThreadEngine & settings) const
{
  return UniquePtr< ::spipe::BoostThreadEngine>::Type(
      new ::spipe::BoostThreadEngine(settings.nThreads));
}
#endif

} // namespace factory
} // namespace spipe
