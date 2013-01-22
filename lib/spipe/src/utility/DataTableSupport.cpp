/*
 * DataTableWriter.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/DataTableSupport.h"

// Local includes
#include "common/SharedData.h"
#include "utility/DataTableWriter.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace utility {
namespace fs = ::boost::filesystem;

DataTableSupport::DataTableSupport(const bool clearTableOnPipeFinish):
myRunner(NULL),
myClearTableOnPipeFinish(clearTableOnPipeFinish)
{}

DataTableSupport::DataTableSupport(
  const ::boost::filesystem::path & filename,
  const bool clearTableOnPipeFinish):
myRunner(NULL),
myFilename(filename),
myClearTableOnPipeFinish(clearTableOnPipeFinish)
{}

DataTableSupport::~DataTableSupport()
{
  deregisterRunner();
}

void DataTableSupport::registerRunner(SpRunnerAccess & runner)
{
  deregisterRunner();

  myRunner = &runner;
  myRunner->addListener(*this);
}

bool DataTableSupport::deregisterRunner()
{
  if(!myRunner)
    return false;

  myRunner->removeListener(*this);
  myRunner = NULL;

  return true;
}

DataTable & DataTableSupport::getTable()
{
  return myTable;
}

void DataTableSupport::setFilename(const fs::path & filename)
{
  if(myFilename == filename)
    return;

  myFilename = filename;

  if(myRunner && myRunner->getState() == ::pipelib::PipelineState::RUNNING)
  {
    if(myWriter.get())
    {
      // Write the final state of the old filename
      myWriter->write();
    }

    // Start the new writer
    createWriter();
  }
}

void DataTableSupport::notify(const ::pipelib::event::PipeRunnerStateChanged<SpRunnerAccess> & evt)
{
  if(evt.getNewState() == ::pipelib::PipelineState::RUNNING)
  {
    createWriter();
  }
  else if(evt.getNewState() == ::pipelib::PipelineState::FINISHED)
  {
    // Write out the final state of the data table
    if(myWriter.get())
      myWriter->write();

    if(myClearTableOnPipeFinish)
      myTable.clear();
  }
}

void DataTableSupport::notify(const ::pipelib::event::PipeRunnerDestroyed<SpRunnerAccess> & evt)
{
  if(myRunner == &evt.getRunner())
    myRunner = NULL;
}

bool DataTableSupport::createWriter()
{
  if(myFilename.empty())
    return false;

  const fs::path outputPath(myRunner->memory().shared().getOutputPath(*myRunner) / myFilename);
  myWriter.reset(new DataTableWriter(myTable, outputPath)); 

  return true;
}

}
}
