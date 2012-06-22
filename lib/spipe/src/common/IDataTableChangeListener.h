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


namespace spipe
{
namespace common
{

class DataTableValueChanged;

class IDataTableChangeListener
{
public:
  virtual void notify(const DataTableValueChanged & evt) = 0;
};

}}

#endif /* I_DATA_TABLE_CHANGE_LISTENER_H */
