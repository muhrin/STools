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

DataTableValueChanged::DataTableValueChanged(const DataTable::Coords & coords,
    const DataTable::Value oldValue, const DataTable::Value & newValue):
    myCoords(coords), myOldValue(oldValue), myNewValue(newValue)
{
}

const ::spipe::utility::DataTable::Coords &
DataTableValueChanged::getCoords() const
{
  return myCoords;
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
