/*
 * LoadSeedStructuresTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spipetest.h"

// From Pipelib
#include <pipelib/DefaultBarrier.h>
#include <pipelib/IDataSink.h>
#include <pipelib/IPipeline.h>
#include <pipelib/SingleThreadedPipeline.h>

// From SSLib
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>

// From SPipe
#include <StructurePipe.h>
#include <common/SharedData.h>
#include <common/StructureData.h>
#include <blocks/LoadSeedStructures.h>


namespace ssc = ::sstbx::common;
namespace blocks = ::spipe::blocks;

class StructureSink : public pipelib::IDataSink<spipe::StructureDataTyp>
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
  typedef ::pipelib::SingleThreadedPipeline<spipe::StructureDataTyp, spipe::SharedDataTyp> Pipe;
  ssc::AtomSpeciesDatabase db;
  StructureSink sink;

  blocks::LoadSeedStructures load(db, "structures/*.res");

  Pipe pipe;
  pipe.setStartBlock(load);
  pipe.setFinishedDataSink(sink);
  pipe.initialise();
  pipe.start();

  BOOST_REQUIRE(sink.getNumReceived() == 10);
}

