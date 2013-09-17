/*
 * DataTableWriter.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/DataTableWriter.h"

#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>

#include "utility/DataTableValueChanged.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace utility {

namespace fs = ::boost::filesystem;

DataTableWriter::DataTableWriter(spipe::utility::DataTable & table, const std::string &filename,
    const bool append, const size_t writeDelay) :
    myTable(table), myAppend(append), myWriteDelay(writeDelay), myOutputPath(
        ::boost::filesystem::path(filename)), myDataSinceWrite(0), myColumnDelimiter(" ")
{
  initialise();
}

DataTableWriter::DataTableWriter(DataTable & table, const ::boost::filesystem::path & filepath,
    const bool append, const size_t writeDelay) :
    myTable(table), myAppend(append), myWriteDelay(writeDelay), myOutputPath(filepath), myDataSinceWrite(
        0), myColumnDelimiter(" ")
{
  initialise();
}

DataTableWriter::~DataTableWriter()
{
  if(myOutStream.is_open())
    myOutStream.close();

  // Tell the table to stop sending us messages
  myTable.removeDataTableChangeListener(*this);
}

bool
DataTableWriter::write()
{
  using ::std::ios_base;

  // Reset changes counter
  myDataSinceWrite = 0;

  if(!myOutStream.is_open())
    myOutStream.open(myOutputPath, ios_base::out | ios_base::in);

  if(myOutStream.good())
  {
    // Move the stream to the correct position
    myOutStream.seekp(myWriteMarker);

    // Print the header first
    myOutStream << "# ";
    for(DataTable::ColumnInfoIterator it = myTable.columnInfoBegin(), end = myTable.columnInfoEnd();
        it != end; ++it)
    {
      myOutStream << it->getName() << myColumnDelimiter;
    }
    myOutStream << "\n";

    // Now print any table notes
    BOOST_FOREACH(const ::std::string & note,
        ::boost::make_iterator_range(myTable.notesBegin(), myTable.notesEnd()))
    {
      myOutStream << "# " << note << "\n";
    }

    for(DataTable::RowIterator it = myTable.rowsBegin(), end = myTable.rowsEnd();
        it != end; ++it)
    {
      BOOST_FOREACH(const DataTable::Value & value, *it)
      {
        myOutStream << value << myColumnDelimiter;
      }
      myOutStream << "\n";
    }
  }
  else
    return false;

  if(myOutStream.is_open())
    myOutStream.close();

  return true;
}

void
DataTableWriter::notify(const DataTableValueChanged & evt)
{
  myDataSinceWrite += diff(evt.getOldValue(), evt.getNewValue());

  if(myDataSinceWrite > myWriteDelay)
  {
    write();
  }
}

void
DataTableWriter::initialise()
{
  using ::std::ios_base;

  const bool fileExists = fs::exists(myOutputPath);

  // Make sure that all the folders up the filename exist
  if(!fileExists)
  {
    const fs::path parentPath = myOutputPath.parent_path();

    if(!parentPath.empty() && !exists(parentPath))
      fs::create_directories(parentPath);
  }

  ::std::ios_base::openmode openMode = ios_base::out;
  if(fileExists && myAppend)
    openMode = openMode | ios_base::app | ios_base::ate;

  // Try opening the file
  myOutStream.open(myOutputPath, openMode);
  // Save the write marker
  myWriteMarker = myOutStream.tellp();
  // Clean up
  myOutStream.close();

  // Listen for changes to the table
  myTable.addDataTableChangeListener(*this);
}

size_t
DataTableWriter::diff(const ::std::string & v1, const ::std::string & v2) const
{
  const size_t min = ::std::min(v1.size(), v2.size());
  size_t numDifferentChars = 0;
  for(size_t i = 0; i < min; ++i)
  {
    if(v1.at(i) != v2.at(i))
      ++numDifferentChars;
  }

  numDifferentChars += ::std::abs((int) v1.size() - (int) v2.size());

  return numDifferentChars;
}

}
}
