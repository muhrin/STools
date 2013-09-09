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


#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/common/Structure.h>

// From SPipe
#include <SpTypes.h>
#include <StructurePipe.h>
#include <common/SharedData.h>
#include <common/StructureData.h>
#include <blocks/KeepTopN.h>

namespace ssc = ::spl::common;
namespace blocks = ::spipe::blocks;
namespace structure_properties = ssc::structure_properties;

class StructureSink : public ::spipe::FinishedSink
{
  typedef ::spipe::StructureDataUniquePtr StructureDataPtr;
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

class StructuresSender : public ::spipe::StartBlock
{
public:
  typedef ::spipe::StructureDataType StructureDataType;

  StructuresSender(const unsigned int numToGenerate):
  ::spipe::Block("Send structures"),
  myNumToGenerate(numToGenerate) {}

  virtual void start()
  {
    typedef ::spl::UniquePtr<ssc::Structure>::Type StructurePtr;
    typedef ::spipe::StructureDataType StructureDataType;
    typedef ::spl::UniquePtr<StructureDataType>::Type StructureDataPtr;

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
      out(getEngine()->registerData(structureData));
    }
  }

private:
  typedef ::std::vector<StructureDataType *> Structures;

  const unsigned int myNumToGenerate;
};

BOOST_AUTO_TEST_CASE(LowestFreeEnergyTest)
{
  typedef spipe::SerialEngine Engine;

  // SETTINGS /////////////
  const size_t NUM_TO_KEEP = 2;
  const unsigned int NUM_STRUCTURES = 10;

  spipe::BlockHandle send(new StructuresSender(NUM_STRUCTURES));
  spipe::BlockHandle keepLowest(new blocks::KeepTopN(NUM_TO_KEEP));

  // Set up the pipe
  send->connect(keepLowest);

  StructureSink sink;

  Engine engine;
  engine.setFinishedDataSink(&sink);
  engine.run(send);

  BOOST_REQUIRE(sink.getNumReceived() == NUM_TO_KEEP);
  // Now try again to make sure it is resetting itself correctly
  sink.reset();
  engine.run(send);
  BOOST_REQUIRE(sink.getNumReceived() == NUM_TO_KEEP);
}

// TODO: Write test to test keep top % mode
