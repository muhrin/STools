/*
 * DataTableWriter.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/DataTableWriter.h"

#include <boost/algorithm/string/join.hpp>
#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>

#include <spl/SSLibAssert.h>

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace utility {

namespace fs = ::boost::filesystem;

DataTableWriter::DataTableWriter(spipe::utility::DataTable & table,
    const std::string &filename, const bool append, const size_t writeDelay) :
    myTable(table), myOutputPath(::boost::filesystem::path(filename)), myAppend(
        append), myColumnDelimiter(" "), myWriteDelay(writeDelay), myDataSinceWrite(
        0)
{
  initialise();
}

DataTableWriter::DataTableWriter(DataTable & table,
    const ::boost::filesystem::path & filepath, const bool append,
    const size_t writeDelay) :
    myTable(table), myOutputPath(filepath), myAppend(append), myColumnDelimiter(
        " "), myWriteDelay(writeDelay), myDataSinceWrite(0)
{
  initialise();
}

DataTableWriter::~DataTableWriter()
{
  if(myOutStream.is_open())
  {
    myOutStream << ::std::endl;
    myOutStream.close();
  }

  // Tell the table to stop sending us messages
  myTable.removeDataTableChangeListener(this);
}

bool
DataTableWriter::write()
{
  // Do we need to write the full table?
  if(myFullWriteRequired)
    return writeFull();
  else
    writeFrom(myLastWriteCoords);
  return true;
}

bool
DataTableWriter::writeFull()
{
  using ::std::ios_base;

  // Reset changes counter
  myDataSinceWrite = 0;

  if(!myOutStream.is_open())
  {
    myOutStream.open(myOutputPath, ios_base::out | ios_base::in);
    // Save the write marker
    myWriteMarker = myOutStream.tellp();
  }

  if(myOutStream.good())
  {
    // Move the stream to the correct position
    myOutStream.seekp(myWriteMarker);

    // Print the header first
    myOutStream << "# "
        << ::boost::algorithm::join(
            ::boost::make_iterator_range(myTable.columnNamesBegin(),
                myTable.columnNamesEnd()), myColumnDelimiter);

    // Now print any table notes
    if(myTable.hasNotes())
    {
      myOutStream << "\n# "
          << ::boost::algorithm::join(
              ::boost::make_iterator_range(myTable.notesBegin(),
                  myTable.notesEnd()), "\n# ");
    }

    for(DataTable::RowIterator it = myTable.rowsBegin(), end =
        myTable.rowsEnd(); it != end; ++it)
    {
      myOutStream << "\n" << ::boost::algorithm::join(*it, myColumnDelimiter);
      myLastWriteCoords.second = it->size();
    }

    ::std::flush(myOutStream);

    myLastWriteCoords.first = myTable.numRows() - 1;

    myFullWriteRequired = false;
  }
  else
    return false;

  return true;
}

void
DataTableWriter::onDataTableValueChanged(const DataTable::Coords & coords,
    const ::std::string & oldValue, const ::std::string & newValue)
{
  myDataSinceWrite += diff(oldValue, newValue);

  if(!myFullWriteRequired && coords < myLastWriteCoords)
    myFullWriteRequired = true;

  if(myDataSinceWrite > myWriteDelay)
    write();
}

void
DataTableWriter::onDataTableColumnChanged(const size_t index, const ::std::string & oldValue,
    const ::std::string & newValue)
{
  myDataSinceWrite += diff(oldValue, newValue);
  myFullWriteRequired = true;

  if(myDataSinceWrite > myWriteDelay)
    write();
}

void
DataTableWriter::initialise()
{
  using ::std::ios_base;

  myLastWriteCoords = DataTable::Coords(0, 0);
  myFullWriteRequired = true;

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
    openMode = openMode | ios_base::ate;

  // Try opening the file
  myOutStream.open(myOutputPath, openMode);
  // Save the write marker
  myWriteMarker = myOutStream.tellp();

  // Listen for changes to the table
  myTable.addDataTableChangeListener(this);
}

void
DataTableWriter::writeFrom(const DataTable::Coords & coords)
{
  // Reset changes counter
  myDataSinceWrite = 0;

  DataTable::RowIterator rowIt = myTable.rowsBegin() + coords.first;

  SSLIB_ASSERT(::std::distance(rowIt, myTable.rowsEnd()) >= 0);

  // is there a remainder of a row to print?
  if(myLastWriteCoords.second > 0 && myLastWriteCoords.second < rowIt->size())
  {
    myOutStream << myColumnDelimiter
        << ::boost::algorithm::join(
            ::boost::make_iterator_range(rowIt->begin() + coords.second,
                rowIt->end()), myColumnDelimiter);
    myLastWriteCoords.second = rowIt->size();
  }

  ++rowIt;
  for(const DataTable::RowIterator end = myTable.rowsEnd(); rowIt != end;
      ++rowIt)
  {
    myOutStream << "\n" << ::boost::algorithm::join(*rowIt, myColumnDelimiter);
    myLastWriteCoords.second = rowIt->size();
  }

  ::std::flush(myOutStream);

  myLastWriteCoords.first = myTable.numRows() - 1;

  myFullWriteRequired = false;
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

  numDifferentChars += ::std::abs(
      static_cast< int>(v1.size()) - static_cast< int>(v2.size()));

  return numDifferentChars;
}

}
}
