/*
 * DataTableSupport.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef DATA_TABLE_SUPPORT_H
#define DATA_TABLE_SUPPORT_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/scoped_ptr.hpp>

#include <pipelib/event/IPipeListener.h>

#include <utility/BoostFilesystem.h>

// Local includes
#include "utility/DataTable.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace pipelib
{
namespace event
{
template <typename Pipe>
class PipeStateChanged;
}
}


namespace spipe
{
namespace utility
{

// FORWARD DECLARATIONS ////////////////////////////////////
class DataTableWriter;

class DataTableSupport : public ::pipelib::event::IPipeListener< ::spipe::SpPipelineTyp>
{
public:

  DataTableSupport(const bool clearTableOnPipeFinish = true);

  DataTableSupport(
    const ::boost::filesystem::path & filename,
    const bool clearTableOnPipeFinish = true);
  ~DataTableSupport();

  void registerPipeline(::spipe::SpPipelineTyp & pipeline);
  bool deregisterPipeline();

  DataTable & getTable();

  void setFilename(const ::boost::filesystem::path & filename);

  // From IPipeListener /////////////////////
  virtual void notify(const ::pipelib::event::PipeStateChanged<SpPipelineTyp> & evt);
  // End from IPipeListener /////////////////

private:

  typedef ::boost::scoped_ptr<DataTableWriter> DataTableWriterPtr;

  bool createWriter();

  ::spipe::SpPipelineTyp *          myPipeline;
  ::boost::filesystem::path         myFilename;
  DataTable                         myTable;
  DataTableWriterPtr                myWriter;
  const bool                        myClearTableOnPipeFinish;

};


}
}

#endif /* DATA_TABLE_SUPPORT_H */
