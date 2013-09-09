/*
 * LoadSeedStructuresTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spipetest.h"

#include <map>
#include <set>
#include <vector>

#include <boost/foreach.hpp>

#include <pipelib/pipelib.h>

#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/common/Structure.h>

// From SPipe
#include <SpTypes.h>
#include <StructurePipe.h>
#include <common/SharedData.h>
#include <common/StructureData.h>
#include <blocks/BuildStructures.h>
#include <blocks/SearchStoichiometries.h>

namespace ssc = ::spl::common;
namespace blocks = ::spipe::blocks;

typedef ::spipe::blocks::SpeciesParameter SpeciesParameter;
typedef ::spipe::blocks::SearchStoichiometries::SpeciesParameters SpeciesParameters;

class StoichiometrySink : public ::spipe::FinishedSink
{
  typedef ::spipe::StructureDataUniquePtr StructureDataPtr;
  typedef ::std::set< size_t> CountSet;
  typedef ::std::map< ssc::AtomSpeciesId::Value, CountSet> SpeciesCount;
public:

  StoichiometrySink(const SpeciesParameters & parameters)
  {
    // Populate with the full count i.e. 0...maxNum
    BOOST_FOREACH(SpeciesParameters::const_reference param, parameters)
    {
      for(size_t i = 0; i < param.maxNum; ++i)
        mySpeciesCount[param.id].insert(i);
    }
  }

  virtual void
  finished(StructureDataPtr structureData)
  {
    typedef ::std::vector< ssc::AtomSpeciesId::Value> Species;

    const ssc::Structure * const structure = structureData->getStructure();
    BOOST_REQUIRE(structure != NULL);

    // Tick off this structure species count
    Species species;
    structure->getAtomSpecies(species);

    SpeciesCount::iterator it;
    CountSet::iterator countIt;
    BOOST_FOREACH(const ssc::AtomSpeciesId::Value & s, species)
    {
      it = mySpeciesCount.find(s);

      // Check that we were expecting this species in the structure
      BOOST_REQUIRE(it != mySpeciesCount.end());

      countIt = it->second.find(structure->getNumAtomsOfSpecies(s));
      BOOST_REQUIRE(countIt != it->second.end());
      it->second.erase(countIt);
    }
  }

  void
  doFinishedCheck() const
  {
    BOOST_FOREACH(SpeciesCount::const_reference speciesCount, mySpeciesCount)
    {
      BOOST_REQUIRE(speciesCount.second.empty());
    }
  }

private:

  SpeciesCount mySpeciesCount;
};

BOOST_AUTO_TEST_CASE(StoichiometrySearchTest)
{
  typedef spipe::SerialEngine Engine;

  // SETTINGS ////
  SpeciesParameters speciesParams;
  speciesParams.push_back(SpeciesParameter("Na", 7));
  speciesParams.push_back(SpeciesParameter("Cl", 13));

  spipe::BlockHandle buildStructures(new blocks::BuildStructures(1));
  spipe::BlockHandle searchStoichiometries(
      new blocks::SearchStoichiometries(speciesParams, 1000, 0.5,
          buildStructures));

  Engine engine;
  StoichiometrySink sink(speciesParams);

  engine.setFinishedDataSink(&sink);
  engine.run(searchStoichiometries);

  sink.doFinishedCheck();
}

