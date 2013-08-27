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
  DataTableValueChanged(const DataTable::Key & key, const DataTable::Column & column,
      const DataTable::Value oldValue, const DataTable::Value & newValue);

  const DataTable::Key &
  getKey() const;
  const DataTable::Column &
  getColumn() const;
  const DataTable::Value &
  getOldValue() const;
  const DataTable::Value &
  getNewValue() const;

private:
  const DataTable::Key & myKey;
  const DataTable::Column & myColumn;
  const DataTable::Value myOldValue;
  const DataTable::Value & myNewValue;

};

}
}

#endif /* I_DATA_TABLE_CHANGE_LISTENER_H */
