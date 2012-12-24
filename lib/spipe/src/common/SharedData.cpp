/*
 * SharedData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/SharedData.h"

#include <pipelib/pipelib.h>

#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;

// Objects keys ////////////////
::sstbx::utility::Key< ::arma::vec> GlobalKeys::POTENTIAL_PARAMS;

const char SharedData::DIR_SUBSTRING_DELIMITER[] = "_";


SharedData::SharedData():
myOutputFileStem(generateUniqueName())
{}

bool SharedData::appendToOutputDirName(const std::string & toAppend)
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

::boost::filesystem::path SharedData::getOutputPath(const SpRunnerAccess & runner) const
{
  ::boost::filesystem::path outPath = runner.memory().global().getPipeRelativeOutputPath();
  // Now build up the from the topmost parent down to this pipeline
  buildOutputPathRecursive(outPath, runner);

  return outPath;
}

const ::boost::filesystem::path & SharedData::getPipeRelativeOutputPath() const
{
  return myOutputDir;
}

const ::boost::filesystem::path & SharedData::getOutputFileStem() const
{
  return myOutputFileStem;
}

ssbc::StructureDescription * SharedData::getStructureDescription()
{
  return structureDescription.get();
}

ssc::AtomSpeciesDatabase & SharedData::getSpeciesDatabase()
{
  return mySpeciesDatabase;
}

const ssc::AtomSpeciesDatabase & SharedData::getSpeciesDatabase() const
{
  return mySpeciesDatabase;
}

void SharedData::reset()
{
  // Reset everything
  potSweepFrom.reset();
  potSweepStep.reset();
  potSweepNSteps.reset();

  structureDescription.reset();

  objectsStore.clear();

  dataTable.clear();

  myOutputDir.clear();

  myOutputFileStem = generateUniqueName();
}

void SharedData::buildOutputPathRecursive(::boost::filesystem::path & path, const SpRunnerAccess & runner) const
{
  const SpRunnerAccess * const parent = runner.getParent();
  if(parent)
  {
    buildOutputPathRecursive(path, *parent);
  }
  path /= runner.memory().shared().getPipeRelativeOutputPath();
}

}
}
