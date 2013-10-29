/*
 * DataTableValueChanged.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef DATA_TABLE_VALUE_CHANGED_H
#define DATA_TABLE_VALUE_CHANGED_H

// INCLUDES /////////////////////////////////////////////
#include <string>

#include "utility/DataTable.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace utility {

class DataTableValueChanged
{
public:
  DataTableValueChanged(const DataTable::Coords & coords,
      const DataTable::Value oldValue, const DataTable::Value & newValue);

  const DataTable::Coords &
  getCoords() const;
  const DataTable::Value &
  getOldValue() const;
  const DataTable::Value &
  getNewValue() const;

private:
  const DataTable::Coords myCoords;
  const DataTable::Value myOldValue;
  const DataTable::Value myNewValue;
};

class ColumnChanged
{
public:
  ColumnChanged()
  {

  }
};

}
}

#endif /* I_DATA_TABLE_CHANGE_LISTENER_H */
