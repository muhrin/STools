/*
 * DataTableWriter.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef DATA_TABLE_WRITER_H
#define DATA_TABLE_WRITER_H

// INCLUDES /////////////////////////////////////////////

#include <map>
#include <string>
#include <vector>

#include <spl/io/BoostFilesystem.h>

// Local includes
#include "utility/IDataTableChangeListener.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace utility {

class DataTable;
class DataTableValueChanged;

class DataTableWriter : public IDataTableChangeListener
{
public:

  static const int DEFAULT_WRITE_DEALY = 50;

  DataTableWriter(DataTable & table, const ::std::string & filename, const bool append = true,
      const size_t writeDelay = DEFAULT_WRITE_DEALY);

  DataTableWriter(DataTable & table, const ::boost::filesystem::path & filepath, const bool append =
      true, const size_t writeDelay = DEFAULT_WRITE_DEALY);

  virtual ~DataTableWriter();

  bool
  write();

  // From IDataTableChangeListener //////////////
  virtual void
  notify(const DataTableValueChanged & evt);
  // End from IDataTableChagneListener ///////////

private:

  void
  initialise();

  size_t
  diff(const ::std::string & v1, const ::std::string & v2) const;

  DataTable & myTable;
  const ::boost::filesystem::path myOutputPath;
  const bool myAppend;
  const ::std::string myColumnDelimiter;
  const size_t myWriteDelay;
  size_t myDataSinceWrite;
  ::boost::filesystem::ofstream myOutStream;
  ::std::streampos myWriteMarker;

};

}
}

#endif /* DATA_TABLE_WRITER_H */
