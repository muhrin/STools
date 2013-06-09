/*
 * LoadSeedStructuresTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spipetest.h"

#include <vector>

#include <boost/foreach.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <pipelib/pipelib.h>

// From SSLib
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>

// From SPipe
#include <SpTypes.h>
#include <StructurePipe.h>
#include <common/SharedData.h>
#include <common/StructureData.h>
#include <blocks/LowestFreeEnergy.h>

namespace ssc = ::sstbx::common;
namespace blocks = ::spipe::blocks;
namespace structure_properties = ssc::structure_properties;

class StructureSink : public ::spipe::SpFinishedSink
{
  typedef ::spipe::SpFinishedSink::PipelineDataPtr StructureDataPtr;
public:

  StructureSink():myNumReceived(0) {}

  void finished(StructureDataPtr data)
  { ++myNumReceived; }

  unsigned int getNumReceived()
  { return myNumReceived; }

  void reset() { myNumReceived = 0; }

private:
  unsigned int myNumReceived;
};

class StructuresSender : public ::spipe::SpStartBlock
{
public:
  typedef ::spipe::StructureDataType StructureDataType;

  StructuresSender(const unsigned int numToGenerate):
  ::spipe::SpStartBlock::BlockType("Send structures"),
  myNumToGenerate(numToGenerate) {}

  virtual void start()
  {
    typedef ::sstbx::UniquePtr<ssc::Structure>::Type StructurePtr;
    typedef ::spipe::StructureDataType StructureDataType;
    typedef ::sstbx::UniquePtr<StructureDataType>::Type StructureDataPtr;

    for(size_t i = 0; i < myNumToGenerate; ++i)
    {
      StructureDataPtr structureData(new StructureDataType());
      StructurePtr structure(new ssc::Structure());
      structure->setProperty(
        structure_properties::general::ENERGY_INTERNAL,
        -static_cast<double>(i) // Set the energy to be the negative of the index (easy to remember)
      );
      structureData->setStructure(structure);
      
      // Register the data and send it on
      out(getRunner()->registerData(structureData));
    }
  }

private:
  typedef ::std::vector<StructureDataType *> Structures;

  const unsigned int myNumToGenerate;
};

BOOST_AUTO_TEST_CASE(LowestFreeEnergyTest)
{
  typedef spipe::SpSingleThreadedEngine Engine;
  typedef spipe::SpPipe Pipe;
  typedef Engine::RunnerPtr RunnerPtr;

  // SETTINGS /////////////
  const size_t NUM_TO_KEEP    = 2;
  const unsigned int NUM_STRUCTURES = 10;

  Pipe pipe;
  StructuresSender * const send = pipe.addBlock(new StructuresSender(NUM_STRUCTURES));
  blocks::LowestFreeEnergy * const lowestEnergy = pipe.addBlock(new blocks::LowestFreeEnergy(NUM_TO_KEEP));

  // Set up the pipe
  pipe.setStartBlock(send);
  pipe.connect(send, lowestEnergy);

  StructureSink sink;


  Engine engine;
  RunnerPtr runner = engine.createRunner();
  runner->setFinishedDataSink(&sink);
  runner->run(pipe);

  BOOST_REQUIRE(sink.getNumReceived() == NUM_TO_KEEP);
  // Now try again to make sure it is resetting itself correctly
  sink.reset();
  runner->run(pipe);
  BOOST_REQUIRE(sink.getNumReceived() == NUM_TO_KEEP);
}

// TODO: Write test to test keep top % mode
