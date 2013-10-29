/*
 * IDataTableChangeListener.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_DATA_TABLE_CHANGE_LISTENER_H
#define I_DATA_TABLE_CHANGE_LISTENER_H

// INCLUDES /////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace utility {

class DataTableValueChanged;

class IDataTableChangeListener
{
public:
  virtual
  ~IDataTableChangeListener()
  {
  }
  virtual void
  onDataTableValueChanged(const DataTable::Coords & coords,
      const ::std::string & oldValue, const ::std::string & newValue)
  {
  }
  virtual void
  onDataTableColumnChanged(const size_t index, const ::std::string & oldValue,
      const ::std::string & newValue)
  {
  }
};

}
}

#endif /* I_DATA_TABLE_CHANGE_LISTENER_H */
