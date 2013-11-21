/*
 * SharedData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/SharedData.h"

#include <pipelib/pipelib.h>

#include <spl/build_cell/StructureGenerator.h>
#include <spl/utility/UtilFunctions.h>

#include "common/GlobalData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace ssbc = ::spl::build_cell;
namespace ssc = ::spl::common;
namespace ssu = ::spl::utility;

const char SharedData::DIR_SUBSTRING_DELIMITER[] = "_";


SharedData::SharedData():
myInstanceName(ssu::generateUniqueName())
{}

void
SharedData::setOutputDir(const ::boost::filesystem::path & path)
{
  myOutputDir = path;
}

bool SharedData::appendToOutputDirName(const std::string & toAppend)
{
  if(toAppend.empty())
    return true;  // Nothing to append

  if(!myOutputDir.empty())
    myOutputDir = myOutputDir.string() + DIR_SUBSTRING_DELIMITER;

  myOutputDir = myOutputDir.string() + toAppend;
  //myOutputDir += toAppend;

  return true;
}

::boost::filesystem::path SharedData::getOutputPath() const
{
  return myOutputDir;
}

const ::std::string & SharedData::getInstanceName() const
{
  return myInstanceName;
}

void SharedData::reset()
{
  // Reset everything
  objectsStore.clear();
  myOutputDir.clear();
  myInstanceName = ssu::generateUniqueName();
}

}
}
