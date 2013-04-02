/*
 * GlobalData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/GlobalData.h"

// From SSTbx
#include <utility/UtilFunctions.h>

#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace ssu = ::sstbx::utility;

namespace spipe {
namespace common {

const char GlobalData::DIR_SUBSTRING_DELIMITER[] = "_";

GlobalData::GlobalData()
{}

bool GlobalData::appendToOutputDirName(const std::string & toAppend)
{
  if(toAppend.empty())
    return true;  // Nothing to append

  if(!myOutputDir.empty())
  {
    myOutputDir = myOutputDir.string() + DIR_SUBSTRING_DELIMITER;
  }
  myOutputDir = myOutputDir.string() + toAppend;

  return true;
}

const ::boost::filesystem::path & GlobalData::getOutputPath() const
{
  return myOutputDir;
}

const ::std::string & GlobalData::getSeedName() const
{
  return mySeedName;
}

void GlobalData::setSeedName(const ::std::string & seedName)
{
  mySeedName = seedName;
}

ssc::AtomSpeciesDatabase & GlobalData::getSpeciesDatabase()
{
  return mySpeciesDatabase;
}

const ssc::AtomSpeciesDatabase & GlobalData::getSpeciesDatabase() const
{
  return mySpeciesDatabase;
}

ssio::StructureReadWriteManager & GlobalData::getStructureIo()
{
  return myStructureIoManager;
}

void GlobalData::reset()
{
  objectsStore.clear();
  myOutputDir.clear();
  mySeedName.clear();
}

}
}
