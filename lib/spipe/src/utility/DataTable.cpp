/*
 * DataTable.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/DataTable.h"

#include <fstream>

#include <boost/foreach.hpp>
#include <boost/algorithm/string/join.hpp>
#ifdef SP_ENABLE_THREAD_AWARE
#  include <boost/thread/lock_guard.hpp>
#endif
#include <boost/range/iterator_range.hpp>

#include <spl/SSLibAssert.h>

#include "utility/IDataTableChangeListener.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace utility {

const ::std::string DataTable::KEY_COLUMN_NAME = "[key]";
const ::std::vector< ::std::string> DataTable::DEFAULT_COLUMN_NAMES(1,
    DataTable::KEY_COLUMN_NAME);

DataTable::DataTable() :
    myColumnNames(DEFAULT_COLUMN_NAMES)
{
}

size_t
DataTable::numRows() const
{
  return myRows.size();
}

DataTable::RowIterator
DataTable::rowsBegin() const
{
  return myRows.begin();
}

DataTable::RowIterator
DataTable::rowsEnd() const
{
  return myRows.end();
}

size_t
DataTable::numColumns() const
{
  return myColumnNames.size();
}

DataTable::ColumnNameIterator
DataTable::columnNamesBegin() const
{
  return myColumnNames.begin();
}

DataTable::ColumnNameIterator
DataTable::columnNamesEnd() const
{
  return myColumnNames.end();
}

bool
DataTable::hasNotes() const
{
  return !myTableNotes.empty();
}

DataTable::NotesIterator
DataTable::notesBegin() const
{
  return myTableNotes.begin();
}

DataTable::NotesIterator
DataTable::notesEnd() const
{
  return myTableNotes.end();
}

DataTable::Coords
DataTable::insert(const DataTable::Key & key, const ::std::string & colName,
    const DataTable::Value & value)
{
#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::lock_guard< boost::mutex> guard(myTableMutex);
#endif

  const Coords coords = this->coords(key, colName);

  const Value oldValue = insertValue(coords, value);

  // Send out message that a value has been inserted
  BOOST_FOREACH(IDataTableChangeListener * l, myListeners)
  {
    l->onDataTableValueChanged(coords, oldValue, value);
  }

  return coords;
}

void
DataTable::addTableNote(const ::std::string & note)
{
#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::lock_guard< boost::mutex> guard(myNotesMutex);
#endif
  myTableNotes.push_back(note);
}

void
DataTable::writeToFile(const ::std::string & filename,
    const ::std::string & colDelimiter) const
{
  ::std::ofstream tableFile;
  tableFile.open(filename.c_str());

  // Print the header first
  tableFile << "# " << ::boost::algorithm::join(myColumnNames, colDelimiter);

  if(hasNotes())
    tableFile << "\n# " << ::boost::algorithm::join(myTableNotes, "\n# ");

  BOOST_FOREACH(const Rows::const_reference row, myRows)
    tableFile << "\n" << ::boost::algorithm::join(row, colDelimiter);

  tableFile << "\n";

  tableFile.close();
}

bool
DataTable::empty() const
{
  if(myRows.empty())
    return true;

  BOOST_FOREACH(const Row & row, myRows)
  {
    if(!row.empty())
      return true;
  }

  return false;
}

void
DataTable::clear()
{
  myRows.clear();
  myRowMap.clear();
  myColumnNames = DEFAULT_COLUMN_NAMES;
  myColumnMap.clear();
  myTableNotes.clear();

  // TODO: Should send message about clearing
}

void
DataTable::addDataTableChangeListener(IDataTableChangeListener * const listener)
{
  myListeners.insert(listener);
}

bool
DataTable::removeDataTableChangeListener(IDataTableChangeListener * const listener)
{
  return myListeners.erase(listener);
}

DataTable::Value
DataTable::insertValue(const Coords & coords, const Value & value)
{
  if(coords.first >= myRows.size())
    myRows.resize(coords.first + 1);
  Row & row = myRows[coords.first];

  Value oldValue;
  // Make sure it's big enough to store the value
  if(coords.second >= row.size())
    row.resize(coords.second + 1);
  else
    oldValue = row[coords.second];

  // Store the value
  row[coords.second] = value;

  return oldValue;
}

size_t
DataTable::row(const Key & key)
{
  SSLIB_ASSERT(!key.empty());

  RowMap::iterator it = myRowMap.find(key);
  if(it == myRowMap.end())
  {
    myRows.push_back(Row(1, key));
    it = myRowMap.insert(::std::make_pair(key, myRows.size() - 1)).first;
  }

  return it->second;
}

size_t
DataTable::col(const ::std::string & name)
{
  SSLIB_ASSERT(!name.empty());

  RowMap::iterator it = myColumnMap.find(name);
  if(it == myColumnMap.end())
  {
    myColumnNames.push_back(name);
    it =
        myColumnMap.insert(::std::make_pair(name, myColumnNames.size() - 1)).first;
    // Send out message that column has been added
    BOOST_FOREACH(IDataTableChangeListener * l, myListeners)
    {
      l->onDataTableColumnChanged(it->second, "", name);
    }
  }

  return it->second;
}

DataTable::Coords
DataTable::coords(const Key & key, const ::std::string & colName)
{
  return Coords(row(key), col(colName));
}

}
}
