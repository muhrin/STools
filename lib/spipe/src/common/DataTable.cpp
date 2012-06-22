/*
 * DataTable.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"
#include "common/DataTable.h"

#include <fstream>

#include <boost/foreach.hpp>

#include "common/DataTableValueChanged.h"
#include "common/IDataTableChangeListener.h"

// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace common
{

DataTable::Column::Column() {}

DataTable::Column::Column(const ::std::string & name):
myName(name)
{}

DataTable::Column::Column(const char * const name):
myName(name)
{}

DataTable::Column::Column(const Column & toCopy):
myName(toCopy.myName)
{}

bool DataTable::Column::operator ==(const Column & rhs) const
{
  return myName == rhs.myName;
}

const ::std::string & DataTable::Column::getName() const
{
  return myName;
}


size_t DataTable::insert(
  const DataTable::Key &    key,
  const DataTable::Column & col,
  const DataTable::Value &  value)
{
  bool found = false;
  int colNo = COL_UNDEFINED;
  for(size_t i = 0; i < myColumns.size(); ++i)
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
  myChangeListenerSupport.notify(DataTableValueChanged(key, col, oldValue, value));

  return colNo;
}

void DataTable::writeToFile(
  const ::std::string & filename,
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
  tableFile << ::std::endl;

  BOOST_FOREACH(const RowMap::value_type & rowPair, myRows)
  {
    tableFile << rowPair.first;
    BOOST_FOREACH(const Value & value, rowPair.second)
    {
      tableFile << colDelimiter << value;
    }
    tableFile << ::std::endl;
  }

  tableFile.close();
}

void DataTable::clear()
{
  myRows.clear();
  myColumns.clear();
}


void DataTable::addDataTableChangeListener(IDataTableChangeListener & listener)
{
  myChangeListenerSupport.insert(listener);
}


bool DataTable::removeDataTableChangeListener(IDataTableChangeListener & listener)
{
  return myChangeListenerSupport.remove(listener);
}

void DataTable::insertColumn(const Column & colInfo, const size_t col)
{
  SP_ASSERT(col != COL_UNDEFINED);

  if(myColumns.size() <= col)
  {
    myColumns.resize(col + 1);
  }

  myColumns[col] = colInfo;
}

DataTable::Value DataTable::insertValue(const Key & key, const size_t col, const Value & value)
{
  ColumnData & colData = myRows[key];

  Value oldValue = "";
  // Make sure it's big enough to store the value
  if(colData.size() <= col)
  {
    colData.resize(col + 1);
  }
  else
  {
    oldValue = colData[col];
  }

  // Store the value
  colData[col] = value;

  return oldValue;
}

}
}
