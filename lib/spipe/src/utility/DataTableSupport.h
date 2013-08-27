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

class DataTableSupport : public SpRunnerListener
{
public:

  DataTableSupport(const bool clearTableOnPipeFinish = true);

  DataTableSupport(
    const ::boost::filesystem::path & filename,
    const bool clearTableOnPipeFinish = true);
  virtual ~DataTableSupport();

  void registerRunner(SpRunnerAccess & runner);
  bool deregisterRunner();

  DataTable & getTable();

  void setFilename(const ::boost::filesystem::path & filename);

  // From IPipeListener /////////////////////
  virtual void notify(const ::pipelib::event::PipeRunnerStateChanged<SpRunnerAccess> & evt);
  virtual void notify(const ::pipelib::event::PipeRunnerDestroyed<SpRunnerAccess> & evt);
  // End from IPipeListener /////////////////

private:

  typedef ::boost::scoped_ptr<DataTableWriter> DataTableWriterPtr;

  bool createWriter();

  SpRunnerAccess *                  myRunner;
  ::boost::filesystem::path         myFilename;
  DataTable                         myTable;
  DataTableWriterPtr                myWriter;
  const bool                        myClearTableOnPipeFinish;

};


}
}

#endif /* DATA_TABLE_SUPPORT_H */
