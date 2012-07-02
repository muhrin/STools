/*
 * DataTableWriter.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/DataTableSupport.h"

#include <pipelib/IPipeline.h>
#include <pipelib/PipelineState.h>
#include <pipelib/event/PipeStateChanged.h>

// Local includes
#include "common/SharedData.h"
#include "utility/DataTableWriter.h"

// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace utility
{
namespace fs = ::boost::filesystem;

DataTableSupport::DataTableSupport(const bool clearTableOnPipeFinish):
myPipeline(NULL),
myClearTableOnPipeFinish(clearTableOnPipeFinish)
{}

DataTableSupport::DataTableSupport(
  const ::boost::filesystem::path & filename,
  const bool clearTableOnPipeFinish):
myPipeline(NULL),
myFilename(filename),
myClearTableOnPipeFinish(clearTableOnPipeFinish)
{}

DataTableSupport::~DataTableSupport()
{
  if(myPipeline)
    myPipeline->removePipeListener(*this);
}

void DataTableSupport::registerPipeline(::spipe::SpPipelineTyp & pipeline)
{
  myPipeline = &pipeline;
  myPipeline->addPipeListener(*this);
}

bool DataTableSupport::deregisterPipeline()
{
  if(!myPipeline)
    return false;

  myPipeline->removePipeListener(*this);
  myPipeline = NULL;

  return true;
}

DataTable & DataTableSupport::getTable()
{
  return myTable;
}

void DataTableSupport::setFilename(const fs::path & filename)
{
  myFilename = filename;

  if(myPipeline && myPipeline->getState() == ::pipelib::PipelineState::RUNNING)
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

void DataTableSupport::notify(const ::pipelib::event::PipeStateChanged<SpPipelineTyp> & evt)
{
  SP_ASSERT(myPipeline == &evt.getPipeline());

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

bool DataTableSupport::createWriter()
{
  if(myFilename.empty())
    return false;

  const fs::path outputPath(myPipeline->getSharedData().getOutputPath() / myFilename);
  myWriter.reset(new DataTableWriter(myTable, outputPath)); 

  return true;
}

}
}
