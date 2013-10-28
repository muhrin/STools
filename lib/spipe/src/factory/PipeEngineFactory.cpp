/*
 * PipeEngineFactory.cpp
 *
 *  Created on: Sep 10, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "factory/PipeEngineFactory.h"

// Local includes
#include "factory/PipeEngineSchemaEntries.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace factory {

PipeEngineFactory::EnginePtr
PipeEngineFactory::createEngine(const HeteroMap & settings) const
{
  const HeteroMap * const serialSettings = settings.find(SERIAL);
  const HeteroMap * const multithreadedSettings = settings.find(BOOST_THREAD);
  if(serialSettings)
    return EnginePtr(createSerialEngine(*serialSettings).release());
#ifdef PIPELIB_USE_BOOST_THREAD
  else if(multithreadedSettings)
    return EnginePtr(createBoostThreadEngine(*multithreadedSettings).release());
#endif

  return EnginePtr();
}

UniquePtr<SerialEngine>::Type
PipeEngineFactory::createSerialEngine(const HeteroMap & settings) const
{
  return UniquePtr<SerialEngine>::Type(new SerialEngine);
}

#ifdef PIPELIB_USE_BOOST_THREAD
UniquePtr<BoostThreadEngine>::Type
PipeEngineFactory::createBoostThreadEngine(const HeteroMap & settings) const
{
  const int * const numThreads = settings.find(NUM_THREADS);
  if(numThreads)
    return UniquePtr<BoostThreadEngine>::Type(new BoostThreadEngine(*numThreads));
  else
    return UniquePtr<BoostThreadEngine>::Type(new BoostThreadEngine());
}
#endif


} // namespace factory
} // namespace spipe
