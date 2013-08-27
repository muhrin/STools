/*
 * DataTableValueChanged.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/DataTableValueChanged.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace utility {

DataTableValueChanged::DataTableValueChanged(const spipe::utility::DataTable::Key & key,
    const spipe::utility::DataTable::Column &column,
    const spipe::utility::DataTable::Value oldValue,
    const spipe::utility::DataTable::Value & newValue) :
    myKey(key), myColumn(column), myOldValue(oldValue), myNewValue(newValue)
{
}

const ::spipe::utility::DataTable::Key &
DataTableValueChanged::getKey() const
{
  return myKey;
}

const ::spipe::utility::DataTable::Column &
DataTableValueChanged::getColumn() const
{
  return myColumn;
}

const ::spipe::utility::DataTable::Value &
DataTableValueChanged::getOldValue() const
{
  return myOldValue;
}

const ::spipe::utility::DataTable::Value &
DataTableValueChanged::getNewValue() const
{
  return myNewValue;
}

}
}
