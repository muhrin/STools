/*
 * LoadSeedStructuresTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spipetest.h"

#include <pipelib/pipelib.h>


#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/common/Structure.h>

// From SPipe
#include <SpTypes.h>
#include <StructurePipe.h>
#include <spl/common/SharedData.h>
#include <spl/common/StructureData.h>
#include <blocks/LoadSeedStructures.h>

namespace ssc = ::spl::common;
namespace blocks = ::spipe::blocks;

class StructureSink : public ::spipe::SpFinishedSink
{
  typedef ::spipe::SpFinishedSink::PipelineDataPtr StructureDataPtr;
public:

  StructureSink():myNumReceived(0) {}

  void finished(StructureDataPtr data)
  { ++myNumReceived; }

  unsigned int getNumReceived()
  { return myNumReceived; }

private:
  unsigned int myNumReceived;
};

BOOST_AUTO_TEST_CASE(LoadSeedStructuresTest)
{
  typedef spipe::SpSingleThreadedEngine Engine;
  typedef spipe::SpPipe Pipe;
  typedef Engine::RunnerPtr RunnerPtr;

  StructureSink sink;

  Pipe pipe;
  blocks::LoadSeedStructures * const load = pipe.addBlock(new blocks::LoadSeedStructures("structures/*.res"));
  pipe.setStartBlock(load);

  Engine engine;
  RunnerPtr runner = engine.createRunner();

  runner->setFinishedDataSink(&sink);
  runner->run(pipe);

  BOOST_REQUIRE(sink.getNumReceived() == 10);
}

