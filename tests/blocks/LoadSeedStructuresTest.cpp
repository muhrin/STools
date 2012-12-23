/*
 * LoadSeedStructuresTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spipetest.h"

// From Pipelib
#include <pipelib/pipelib.h>

// From SSLib
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>

// From SPipe
#include <PipeLibTypes.h>
#include <StructurePipe.h>
#include <common/SharedData.h>
#include <common/StructureData.h>
#include <blocks/LoadSeedStructures.h>

namespace ssc = ::sstbx::common;
namespace blocks = ::spipe::blocks;

class StructureSink : public ::spipe::SpFinishedSink
{
public:
  StructureSink():myNumReceived(0) {}

  void in(spipe::StructureDataTyp * const data)
  {
    ++myNumReceived;
  }

  unsigned int getNumReceived()
  { return myNumReceived; }

private:
  unsigned int myNumReceived;
};

BOOST_AUTO_TEST_CASE(LoadSeedStructuresTest)
{
  //TODO: port to new api
/*  typedef ::pipelib::SingleThreadedEngine Engine;
  ssc::AtomSpeciesDatabase db;
  StructureSink sink;

  blocks::LoadSeedStructures load(db, "structures/*.res");

  pipe.setStartBlock(load);

  Engine engine;
  SpPipeRunner runner = engine.create
  pipe.setFinishedDataSink(sink);
  pipe.initialise();
  pipe.start();

  BOOST_REQUIRE(sink.getNumReceived() == 10);*/
}

