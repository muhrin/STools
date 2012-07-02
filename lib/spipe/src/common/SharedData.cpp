/*
 * SharedData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/SharedData.h"

#include <pipelib/IPipeline.h>
#include <pipelib/PipelineState.h>
#include <pipelib/event/PipeStateChanged.h>

#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace common
{

// Objects keys ////////////////
const ::sstbx::utility::Key< ::arma::vec> GlobalKeys::POTENTIAL_PARAMS;

const char * SharedData::DIR_SUBSTRING_DELIMITER = "_";

SharedData::SharedData():
myPipe(NULL),
myOutputFileStem(generateUniqueName())
{}

SharedData::~SharedData()
{
  // Make sure to tell that pipe that we no longer want to receive messages
  if(myPipe)
    myPipe->removePipeListener(*this);
}

void SharedData::setPipe(spipe::SpPipelineTyp & pipe)
{
  // Precondition: myPipe has not be null.
  SP_ASSERT(!myPipe);

  myPipe = &pipe;
  myPipe->addPipeListener(*this);
}

bool SharedData::appendToOutputDirName(const std::string & toAppend)
{
  if(toAppend.empty())
    return true;  // Nothing to append

  if(myPipe->getState() == ::pipelib::PipelineState::RUNNING)
    return false;  // Can't change output directory while the pipeline is running!

  if(!myOutputDir.empty())
  {
    myOutputDir = myOutputDir.string() + DIR_SUBSTRING_DELIMITER;
  }
  myOutputDir = myOutputDir.string() + toAppend;

  return true;
}

::boost::filesystem::path SharedData::getOutputPath() const
{
  SP_ASSERT(myPipe);

  ::boost::filesystem::path outPath = myPipe->getGlobalData().getRelativeOutputPath();
  // Now build up the from the topmost parent down to this pipeline
  buildOutputPathRecursive(outPath, *myPipe);

  return outPath;
}

const ::boost::filesystem::path & SharedData::getOutputFileStem() const
{
  return myOutputFileStem;
}

const ::boost::filesystem::path & SharedData::getRelativeOutputPath() const
{
  return myOutputDir;
}

void SharedData::notify(const ::pipelib::event::PipeStateChanged< ::spipe::SpPipelineTyp> & evt)
{
  SP_ASSERT(&evt.getPipeline() == myPipe);

  if(evt.getNewState() == ::pipelib::PipelineState::FINISHED)
  {
    reset();
  }
}

void SharedData::reset()
{
  // Reset everything
  potSweepFrom.reset();
  potSweepStep.reset();
  potSweepNSteps.reset();

  structureDescription.reset();
  cellDescription.reset();

  objectsStore.clear();

  dataTable.clear();

  myOutputDir.clear();

  myOutputFileStem = generateUniqueName();
}

void SharedData::buildOutputPathRecursive(::boost::filesystem::path & path, const ::spipe::SpPipelineTyp & pipe) const
{
  const ::spipe::SpPipelineTyp * const parent = pipe.getParent();
  if(parent)
  {
    buildOutputPathRecursive(path, *parent);
  }
  path /= pipe.getSharedData().getRelativeOutputPath();
}

}
}
