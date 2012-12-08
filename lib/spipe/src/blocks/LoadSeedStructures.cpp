/*
 * RandomStructure.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/LoadSeedStructures.h"

#include <sstream>
#include <vector>

#include <boost/foreach.hpp>

// From SSTbx
#include <common/AtomSpeciesDatabase.h>
#include <common/Constants.h>
#include <common/Structure.h>
#include <io/IoFunctions.h>

// From PipelineLib
#include <pipelib/IPipeline.h>

// Local includes
#include "common/SharedData.h"
#include "common/StructureData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace fs = ::boost::filesystem;
namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;

const double LoadSeedStructures::ATOMIC_VOLUME_MULTIPLIER = 2.0;

LoadSeedStructures::LoadSeedStructures(
  const ssc::AtomSpeciesDatabase & atomSpeciesDb,
  const ::std::string & seedStructures,
  const bool tryToScaleVolumes):
::pipelib::Block<spipe::StructureDataTyp, spipe::SharedDataTyp>("Load seed structures"),
mySpeciesDb(atomSpeciesDb),
myTryToScaleVolumes(tryToScaleVolumes)
{
  // First of all split the string up
  ::std::string entry;
  std::stringstream stream(seedStructures);
  while(getline(stream, entry))
    processEntry(entry);
}


void LoadSeedStructures::start()
{
	using ::spipe::common::StructureData;

  double oldVolume, newVolume;
  const ssc::UnitCell * unitCell;
  BOOST_FOREACH(const ssc::Structure & str, myStructures)
  {
    StructureData & data = myPipeline->newData();
    // Make a clone of our structure
    ssc::Structure & structure = data.setStructure(str.clone());
    
    // Set up the structure name if needed
    if(structure.getName().empty())
      structure.setName(common::generateUniqueName());

    unitCell = structure.getUnitCell();
    if(myTryToScaleVolumes && unitCell)
    {
      oldVolume = unitCell->getVolume();
      newVolume = ATOMIC_VOLUME_MULTIPLIER * getTotalAtomicVolume(str);
      structure.scale(newVolume / oldVolume);
    }

    // Send it on its way
    myOutput->in(data);
  }
}

int LoadSeedStructures::processEntry(const ::std::string & entry)
{
  const EntryType type = entryType(entry);

  if(type == FILE_PATH)
    return processFilePath(fs::path(entry));
  else if(type == FOLDER_PATH)
    return processFolderPath(fs::path(entry));
  else if(type == WILDCARD_PATH)
    return processWildcardEntry(entry);

  return -1;
}

int LoadSeedStructures::processWildcardEntry(const ::std::string & entry)
{
  ::std::vector<fs::path> entryPaths;

  if(!ssio::getWildcardPaths(entry, entryPaths))
    return -1;

  int numOut = 0;

  BOOST_FOREACH(const fs::path & entryPath, entryPaths)
  {
    if(fs::is_regular_file(entryPath))
      numOut += processFilePath(entryPath);
  }

  return numOut;
}

int LoadSeedStructures::processFilePath(const boost::filesystem::path & entryPath)
{
  // Try loading the file
  const size_t numLoaded = myReader.readStructures(myStructures, entryPath, mySpeciesDb);
  if(numLoaded > 0)
    return (int)numLoaded;

  return -1;
}

int LoadSeedStructures::processFolderPath(const boost::filesystem::path & entryPath)
{
  ::std::vector<fs::path> entryPaths;

  if(!ssio::getWildcardPaths(".*\\.res", entryPaths, fs::current_path()))
    return -1;

  int numOut = 0;

  BOOST_FOREACH(const fs::path & entryPath, entryPaths)
  {
    if(fs::is_regular_file(entryPath))
      numOut += processFilePath(entryPath);
  }

  return numOut;
}

LoadSeedStructures::EntryType
LoadSeedStructures::entryType(const ::std::string & entry) const
{
  if(entry.find('*') != ::std::string::npos)
    return WILDCARD_PATH;

  const fs::path entryPath(entry);
  if(fs::exists(entryPath))
  {
    if(fs::is_regular_file(entryPath))
      return FILE_PATH;
    else if(fs::is_directory(entryPath))
      return FOLDER_PATH;
  }

  return UNKNOWN;
}

double LoadSeedStructures::getTotalAtomicVolume(
  const ::sstbx::common::Structure & structure) const
{
  typedef ::boost::optional<double> OptionalDouble;

  ::std::vector<ssc::AtomSpeciesId::Value> species;
  structure.getAtomSpecies(species);

  OptionalDouble radius;
  double dRadius, volume = 0.0;
  BOOST_FOREACH(ssc::AtomSpeciesId::Value spec, species)
  {
    radius = mySpeciesDb.getRadius(spec);
    if(radius)
    {
      dRadius = *radius;
      volume += dRadius * dRadius * dRadius;
    }
  }

  volume *= (4.0 / 3.0) * ssc::Constants::PI;
  return volume;
}

}
}
