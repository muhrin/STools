/*
 * SharedData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/SharedData.h"

#include <pipelib/pipelib.h>

#include <spl/utility/UtilFunctions.h>

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace ssu = ::spl::utility;

SharedData::SharedData()
{
  reset();
}

const ::std::string & SharedData::getInstanceName() const
{
  return myInstanceName;
}

spl::common::AtomSpeciesDatabase &
SharedData::getSpeciesDatabase()
{
  return mySpeciesDb;
}

const spl::common::AtomSpeciesDatabase &
SharedData::getSpeciesDatabase() const
{
  return mySpeciesDb;
}


const ::boost::filesystem::path &
SharedData::getWorkingDir() const
{
  return myWorkingDir;
}

void
SharedData::setWorkingDir(const ::boost::filesystem::path & workingDir)
{
  myWorkingDir = workingDir;
}

void SharedData::reset()
{
  // Reset everything
  objectsStore.clear();
  myWorkingDir.clear();
  myInstanceName = ssu::generateUniqueName();
}

}
}
