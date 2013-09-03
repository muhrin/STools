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

#include <spl/io/BoostFilesystem.h>

#include <pipelib/pipelib.h>

// Local includes
#include "SpTypes.h"
#include "utility/DataTable.h"

namespace spipe {
namespace utility {

// FORWARD DECLARATIONS ////////////////////////////////////
class DataTableWriter;

class DataTableSupport : public EngineAccessListener
{
public:

  DataTableSupport(const bool clearTableOnPipeFinish = true);

  DataTableSupport(const ::boost::filesystem::path & filename, const bool clearTableOnPipeFinish =
      true);
  virtual
  ~DataTableSupport();

  void
  registerEngine(EngineAccess * const engine);
  bool
  deregisterEngine();

  DataTable &
  getTable();

  void
  setFilename(const ::boost::filesystem::path & filename);

  // From IPipeListener /////////////////////
  virtual void
  notify(const ::pipelib::event::PipeEngineStateChanged< EngineAccess> & evt);
  virtual void
  notify(const ::pipelib::event::PipeEngineDestroyed< EngineAccess> & evt);
  // End from IPipeListener /////////////////

private:
  typedef ::boost::scoped_ptr< DataTableWriter> DataTableWriterPtr;

  bool
  createWriter();

  EngineAccess * myEngine;
  ::boost::filesystem::path myFilename;
  DataTable myTable;
  DataTableWriterPtr myWriter;
  const bool myClearTableOnPipeFinish;

};

}
}

#endif /* DATA_TABLE_SUPPORT_H */
