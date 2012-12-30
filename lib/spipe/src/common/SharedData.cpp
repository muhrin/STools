/*
 * SharedData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/SharedData.h"

#include <pipelib/pipelib.h>

// From SSTbx
#include <utility/UtilFunctions.h>

#include "common/GlobalData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;
namespace ssu = ::sstbx::utility;

const char SharedData::DIR_SUBSTRING_DELIMITER[] = "_";


SharedData::SharedData():
myOutputFileStem(ssu::generateUniqueName())
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

::boost::filesystem::path SharedData::getOutputPath(const SpRunner & runner) const
{
  ::boost::filesystem::path outPath = runner.memory().global().getOutputPath();
  // Now build up the from the topmost parent down to this pipeline
  buildOutputPathRecursive(outPath, runner);

  return outPath;
}

::boost::filesystem::path SharedData::getOutputPath(const SpRunnerAccess & runner) const
{
  ::boost::filesystem::path outPath = runner.memory().global().getOutputPath();
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

SharedData::StructureDescriptionPtr SharedData::getStructureDescription()
{
  return structureDescription;
}

SharedData::ConstStructureDescriptionPtr SharedData::getStructureDescription() const
{
  return structureDescription;
}

void SharedData::setStructureDescription(StructureDescriptionPtr description)
{
  structureDescription = description;
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
  structureDescription.reset();
  objectsStore.clear();
  dataTable.clear();
  myOutputDir.clear();
  myOutputFileStem = ssu::generateUniqueName();
}

void SharedData::buildOutputPathRecursive(::boost::filesystem::path & path, const SpRunner & runner) const
{
  const SpRunner * const parent = runner.getParent();
  if(parent)
  {
    buildOutputPathRecursive(path, *parent);
  }
  path /= runner.memory().shared().getPipeRelativeOutputPath();
}

void SharedData::buildOutputPathRecursive(::boost::filesystem::path & path, const SpRunnerAccess & runner) const
{
  const SpRunnerAccess * const parent = runner.getParentAccess();
  if(parent)
  {
    buildOutputPathRecursive(path, *parent);
  }
  path /= runner.memory().shared().getPipeRelativeOutputPath();
}

}
}
