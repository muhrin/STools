/*
 * GlobalData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/GlobalData.h"

#include <spl/utility/UtilFunctions.h>

#include "common/UtilityFunctions.h"
#include "utility/PipeDataInitialisation.h"

// NAMESPACES ////////////////////////////////
namespace ssc = ::spl::common;
namespace ssio = ::spl::io;
namespace ssu = ::spl::utility;

namespace spipe {
namespace common {

GlobalData::GlobalData()
{
  utility::initStructureRwManDefault(myStructureIoManager);
}

const ::std::string &
GlobalData::getSeedName() const
{
  return mySeedName;
}

void
GlobalData::setSeedName(const ::std::string & seedName)
{
  mySeedName = seedName;
}

ssc::AtomSpeciesDatabase &
GlobalData::getSpeciesDatabase()
{
  return mySpeciesDatabase;
}

const ssc::AtomSpeciesDatabase &
GlobalData::getSpeciesDatabase() const
{
  return mySpeciesDatabase;
}

ssio::StructureReadWriteManager &
GlobalData::getStructureIo()
{
  return myStructureIoManager;
}

const ::boost::filesystem::path &
GlobalData::getWorkingDir() const
{
  return myWorkingDir;
}

void
GlobalData::setWorkingDir(const ::boost::filesystem::path & workingDir)
{
  myWorkingDir = workingDir;
}

void
GlobalData::reset()
{
  objectsStore.clear();
  myWorkingDir.clear();
  mySeedName.clear();
}

}
}
