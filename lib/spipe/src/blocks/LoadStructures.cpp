/*
 * LoadStructures.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/LoadStructures.h"

#include <iterator>
#include <sstream>
#include <vector>

#include <boost/foreach.hpp>

#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/common/Constants.h>
#include <spl/common/Structure.h>
#include <spl/io/IoFunctions.h>
#include <spl/io/ResourceLocator.h>
#include <spl/utility/UtilFunctions.h>

// Local includes
#include "common/PipeFunctions.h"
#include "common/SharedData.h"
#include "common/StructureData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

namespace fs = ::boost::filesystem;
namespace ssbc = ::spl::build_cell;
namespace ssc = ::spl::common;
namespace ssio = ::spl::io;
namespace ssu = ::spl::utility;

const double LoadStructures::ATOMIC_VOLUME_MULTIPLIER = 2.0;

LoadStructures::LoadStructures(const ::std::string & seedStructures,
    const bool tryToScaleVolumes) :
    Block("Load structures"), mySeedStructuresString(seedStructures), myTryToScaleVolumes(
        tryToScaleVolumes)
{
}

void
LoadStructures::pipelineInitialising()
{
  std::queue< ::spl::io::ResourceLocator> empty;
  std::swap(myStructureLocations, empty);

  // First of all split the string up
  ::std::string entry;
  std::stringstream stream(mySeedStructuresString);
  while(getline(stream, entry))
    processEntry(entry);
}

void
LoadStructures::start()
{
  using ::spipe::common::StructureData;

  double oldVolume, newVolume;
  const ssc::UnitCell * unitCell;

  while(!myStructureLocations.empty())
  {
    const ssio::ResourceLocator & loc = myStructureLocations.back();

    // Load the structures
    ssio::StructuresContainer structures;
    getEngine()->globalData().getStructureIo().readStructures(structures, loc);

    while(!structures.empty())
    {
      StructureData * const data = getEngine()->createData();
      ssc::Structure & structure = data->setStructure(structures.pop_back());

      // Set up the structure name if needed
      if(structure.getName().empty())
        structure.setName(
            common::generateStructureName(getEngine()->globalData()));

      unitCell = structure.getUnitCell();
      if(myTryToScaleVolumes && unitCell)
      {
        oldVolume = unitCell->getVolume();
        newVolume = ATOMIC_VOLUME_MULTIPLIER * getTotalAtomicVolume(structure);
        structure.scale(newVolume / oldVolume);
      }

      // Send it on its way
      out(data);
    }

    myStructureLocations.pop(); // Pop off the queue of structures to load
  }
}

int
LoadStructures::processEntry(const ::std::string & entry)
{
  const EntryType type = entryType(entry);

  if(type == FILE_PATH || type == FOLDER_PATH)
  {
    ssio::ResourceLocator loc;
    loc.set(entry);
    myStructureLocations.push(loc);
  }
  else if(type == WILDCARD_PATH)
    return processWildcardEntry(entry);

  return -1;
}

int
LoadStructures::processWildcardEntry(const ::std::string & entry)
{
  ::std::vector< fs::path> entryPaths;

  if(!ssio::getWildcardPaths(entry, entryPaths))
    return -1;

  int numOut = 0;
  BOOST_FOREACH(const fs::path & entryPath, entryPaths)
  {
    if(fs::is_regular_file(entryPath))
    {
      myStructureLocations.push(ssio::ResourceLocator(entryPath));
      ++numOut;
    }
  }
  return numOut;
}

LoadStructures::EntryType
LoadStructures::entryType(const ::std::string & entry) const
{
  if(entry.find('*') != ::std::string::npos)
    return WILDCARD_PATH;

  ssio::ResourceLocator entryLocator;
  if(!entryLocator.set(entry))
    return UNKNOWN;

  if(fs::exists(entryLocator.path()))
  {
    if(fs::is_regular_file(entryLocator.path()))
      return FILE_PATH;
    else if(fs::is_directory(entryLocator.path()))
      return FOLDER_PATH;
  }

  return UNKNOWN;
}

double
LoadStructures::getTotalAtomicVolume(
    const ::spl::common::Structure & structure) const
{
  typedef ::boost::optional< double> OptionalDouble;

  ::std::vector< ssc::AtomSpeciesId::Value> species;
  structure.getAtomSpecies(::std::back_inserter(species));

  OptionalDouble radius;
  double dRadius, volume = 0.0;
  const ssc::AtomSpeciesDatabase & speciesDb =
      getEngine()->sharedData().getSpeciesDatabase();
  BOOST_FOREACH(ssc::AtomSpeciesId::Value spec, species)
  {
    radius = speciesDb.getRadius(spec);
    if(radius)
    {
      dRadius = *radius;
      volume += dRadius * dRadius * dRadius;
    }
  }

  volume *= ssc::constants::FOUR_THIRDS * ssc::constants::PI;
  return volume;
}

}
}
