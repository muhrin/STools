/*
 * DataTable.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"
#include "utility/DataTable.h"

#include <fstream>

#include <boost/foreach.hpp>

#include "utility/DataTableValueChanged.h"
#include "utility/IDataTableChangeListener.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace utility {

DataTable::Column::Column()
{
}

DataTable::Column::Column(const ::std::string & name) :
    myName(name)
{
}

DataTable::Column::Column(const char * const name) :
    myName(name)
{
}

DataTable::Column::Column(const Column & toCopy) :
    myName(toCopy.myName)
{
}

bool
DataTable::Column::operator ==(const Column & rhs) const
{
  return myName == rhs.myName;
}

const ::std::string &
DataTable::Column::getName() const
{
  return myName;
}

DataTable::DataTable():
    myColumns(1, Column("[key]"))
{}

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

DataTable::ColumnInfoIterator DataTable::columnInfoBegin() const
{
  return myColumns.begin();
}

DataTable::ColumnInfoIterator DataTable::columnInfoEnd() const
{
  return myColumns.end();
}

DataTable::NotesIterator DataTable::notesBegin() const
{
  return myTableNotes.begin();
}

DataTable::NotesIterator DataTable::notesEnd() const
{
  return myTableNotes.end();
}

size_t
DataTable::insert(const DataTable::Key & key, const DataTable::Column & col,
    const DataTable::Value & value)
{
#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::lock_guard<boost::mutex> guard(myTableMutex);
#endif

  int colNo = COL_UNDEFINED;
  for(size_t i = 1; i < myColumns.size(); ++i)
  {
    if(myColumns[i] == col)
    {
      colNo = i;
      break;
    }
  }

  if(colNo == COL_UNDEFINED)
  {
    // Stick the column at the end
    colNo = myColumns.size();
    insertColumn(col, colNo);
  }

  const Value oldValue = insertValue(key, colNo, value);

  // Send out message that a value has been inserted
  myChangeListenerSupport.notify(
      DataTableValueChanged(key, col, oldValue, value));

  return colNo;
}

void
DataTable::addTableNote(const ::std::string & note)
{
#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::lock_guard<boost::mutex> guard(myNotesMutex);
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
  tableFile << "# " << "[key]";
  BOOST_FOREACH(const DataTable::Column & colInfo, myColumns)
  {
    tableFile << colDelimiter << colInfo.getName();
  }
  tableFile << "\n";

  BOOST_FOREACH(const Rows::const_reference row, myRows)
  {
    BOOST_FOREACH(const Value & value, row)
    {
      tableFile << colDelimiter << value;
    }
    tableFile << "\n";
  }

  tableFile.close();
}

void
DataTable::clear()
{
  myRows.clear();
  myColumns.clear();
  myTableNotes.clear();
}

void
DataTable::addDataTableChangeListener(IDataTableChangeListener & listener)
{
  myChangeListenerSupport.insert(listener);
}

bool
DataTable::removeDataTableChangeListener(IDataTableChangeListener & listener)
{
  return myChangeListenerSupport.remove(listener);
}

void
DataTable::insertColumn(const Column & colInfo, const size_t col)
{
  if(myColumns.size() <= col)
    myColumns.resize(col + 1);

  myColumns[col] = colInfo;
}

DataTable::Value
DataTable::insertValue(const Key & key, const size_t col, const Value & value)
{
  RowMap::iterator it = myRowMap.find(key);

  if(it == myRowMap.end())
  {
    myRows.push_back(ColumnData(1, key));
    it = myRowMap.insert(::std::make_pair(key, myRows.size() - 1)).first;
  }

  ColumnData & colData = myRows[it->second];

  Value oldValue = "";
  // Make sure it's big enough to store the value
  if(colData.size() <= col)
    colData.resize(col + 1);
  else
    oldValue = colData[col];

  // Store the value
  colData[col] = value;

  return oldValue;
}

}
}
