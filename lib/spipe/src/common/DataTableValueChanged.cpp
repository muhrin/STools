/*
 * DataTableValueChanged.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/DataTableValueChanged.h"

// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace common
{

DataTableValueChanged::DataTableValueChanged(
  const spipe::common::DataTable::Key &   key,
  const spipe::common::DataTable::Column &column,
  const spipe::common::DataTable::Value   oldValue,
  const spipe::common::DataTable::Value & newValue):
myKey(key),
myColumn(column),
myOldValue(oldValue),
myNewValue(newValue)
{}

const ::spipe::common::DataTable::Key & DataTableValueChanged::getKey() const
{
  return myKey;
}

const ::spipe::common::DataTable::Column & DataTableValueChanged::getColumn() const
{
  return myColumn;
}

const ::spipe::common::DataTable::Value & DataTableValueChanged::getOldValue() const
{
  return myOldValue;
}

const ::spipe::common::DataTable::Value & DataTableValueChanged::getNewValue() const
{
  return myNewValue;
}






}
}
