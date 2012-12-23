/*
 * RandomStructure.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/LoadSeedStructures.h"

#include <sstream>

#include <boost/foreach.hpp>

// From SSTbx
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>
#include <io/IoFunctions.h>

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

LoadSeedStructures::LoadSeedStructures(
  const ssc::AtomSpeciesDatabase & atomSpeciesDb,
  const ::std::string & seedStructures):
::pipelib::Block<StructureDataTyp, SharedDataTyp, SharedDataTyp>("Load seed structures"),
mySpeciesDb(atomSpeciesDb)
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

  BOOST_FOREACH(const ssc::Structure & str, myStructures)
  {
    StructureData & data = getRunner()->createData();
    // Make a clone of our structure
    ssc::Structure & structure = data.setStructure(str.clone());
    
    // Set up the structure name
    if(structure.getName().empty())
      structure.setName(common::generateUniqueName());
    data.name.reset(structure.getName());

    // Send it on its way
    out(data);
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
  StructurePtr str = myReader.readStructure(entryPath, mySpeciesDb);
  if(str.get())
  {
    myStructures.push_back(str.release());
    return 1;
  }
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

}
}
