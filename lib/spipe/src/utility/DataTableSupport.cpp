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
myEngine(NULL),
myClearTableOnPipeFinish(clearTableOnPipeFinish)
{}

DataTableSupport::DataTableSupport(
  const ::boost::filesystem::path & filename,
  const bool clearTableOnPipeFinish):
myEngine(NULL),
myFilename(filename),
myClearTableOnPipeFinish(clearTableOnPipeFinish)
{}

DataTableSupport::~DataTableSupport()
{
  deregisterEngine();
}

void DataTableSupport::registerEngine(EngineAccess * const engine)
{
  deregisterEngine();

  myEngine = engine;
  myEngine->addListener(*this);
}

bool DataTableSupport::deregisterEngine()
{
  if(!myEngine)
    return false;

  myEngine->removeListener(*this);
  myEngine = NULL;

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

  if(myEngine && myEngine->getState() == ::pipelib::PipelineState::RUNNING)
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

void DataTableSupport::notify(const ::pipelib::event::PipeEngineStateChanged<EngineAccess> & evt)
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

void DataTableSupport::notify(const ::pipelib::event::PipeEngineDestroyed<EngineAccess> & evt)
{
  if(myEngine == &evt.getEngine())
    myEngine = NULL;
}

bool DataTableSupport::createWriter()
{
  if(myFilename.empty())
    return false;

  myWriter.reset(new DataTableWriter(myTable, fs::path(myFilename)));
  return true;
}

}
}
