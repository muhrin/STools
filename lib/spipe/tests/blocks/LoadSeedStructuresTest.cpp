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
#include <common/SharedData.h>
#include <common/StructureData.h>
#include <blocks/LoadStructures.h>

namespace ssc = ::spl::common;
namespace blocks = ::spipe::blocks;

class StructureSink : public ::spipe::FinishedSink
{
  typedef ::spipe::StructureDataUniquePtr StructureDataPtr;
public:

  StructureSink():myNumReceived(0) {}

  virtual void finished(StructureDataPtr data)
  { ++myNumReceived; }

  unsigned int getNumReceived()
  { return myNumReceived; }

private:
  unsigned int myNumReceived;
};

BOOST_AUTO_TEST_CASE(LoadSeedStructuresTest)
{
  typedef spipe::SerialEngine Engine;

  StructureSink sink;

  spipe::BlockHandle load(new blocks::LoadStructures("structures/*.res"));

  Engine engine;
  engine.setFinishedDataSink(&sink);
  engine.run(load);

  BOOST_REQUIRE(sink.getNumReceived() == 10);
}

