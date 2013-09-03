/*
 * SharedData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/SharedData.h"

#include <pipelib/pipelib.h>


#include <spl/build_cell/IStructureGenerator.h>
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

bool SharedData::appendToOutputDirName(const std::string & toAppend)
{
  if(toAppend.empty())
    return true;  // Nothing to append

  if(!myOutputDir.empty())
    myOutputDir = myOutputDir.string() + DIR_SUBSTRING_DELIMITER;

  myOutputDir = myOutputDir.string() + toAppend;

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

ssbc::IStructureGenerator * SharedData::getStructureGenerator()
{
  return myStructureGenerator.get();
}

const ssbc::IStructureGenerator * SharedData::getStructureGenerator() const
{
  return myStructureGenerator.get();
}

void SharedData::reset()
{
  // Reset everything
  myStructureGenerator.reset();
  objectsStore.clear();
  myOutputDir.clear();
  myInstanceName = ssu::generateUniqueName();
}

}
}
