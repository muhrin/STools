/*
 * InfoToken.h
 *
 *
 *  Created on: Nov 19, 2012
 *      Author: Martin Uhrin
 */

#ifndef INFO_TOKEN_DETAIL_H
#define INFO_TOKEN_DETAIL_H

// INCLUDES /////////////////////////////////////////////

#include <common/Structure.h>

// FORWARD DECLARES ////////////////////////////////
namespace stools {
namespace utility {

template <typename T>
TypedToken<T>::TypedToken(
  const ::std::string & name,
  const ::std::string & symbol,
  const ::std::string & defaultFormatString):
InfoToken(symbol, defaultFormatString),
myColumn(name)
{}

template <typename T>
::std::string TypedToken<T>::getName() const
{
  return myColumn.getName();
}

template <typename T>
bool TypedToken<T>::insert(
  StructureInfoTable & table,
  const ::sstbx::common::Structure & structure)
{
  StructureValue value = doGetValue(structure);
  
  if(!value)
    return false; 

  table.set(&structure, myColumn, *value);
  return true;
}

template <typename T>
bool TypedToken<T>::remove(StructureInfoTable & table)
{
  // TODO: Implement
  return false;
}

template <typename T>
void TypedToken<T>::sort(SortedKeys & keys, const StructureInfoTable & table) const
{
  table.getAscending(keys, myColumn);
}

template <typename T>
const typename TypedToken<T>::Column &
TypedToken<T>::getColumn() const
{
  return myColumn;
}

template <typename T>
StructurePropertyToken<T>::StructurePropertyToken(
  const ::std::string & name,
  const ::std::string & symbol,
  PropertyKey & propertyKey,
  const ::std::string & defaultFormatString):
TypedToken<T>(name, symbol, defaultFormatString),
myKey(propertyKey)
{}

template <typename T>
typename StructurePropertyToken<T>::StructureValue
StructurePropertyToken<T>::doGetValue(const ::sstbx::common::Structure & structure) const
{
  StructureValue value;
  const T * const valuePtr = structure.getProperty(myKey);
  
  if(valuePtr)
    value.reset(*valuePtr);

  return value;
}

template <typename T, typename Getter>
FunctionToken<T, Getter>::FunctionToken(
  const ::std::string & name,
  const ::std::string & symbol,
  Getter getter,
  const ::std::string & defaultFormatString):
TypedToken<T>(name, symbol, defaultFormatString),
myGetter(getter)
{}

template <typename T, typename Getter>
typename FunctionToken<T, Getter>::StructureValue
FunctionToken<T, Getter>::doGetValue(const ::sstbx::common::Structure & structure) const
{
  return myGetter(structure);
}

template <typename T>
::std::auto_ptr<InfoToken>
makeStructurePropertyToken(
  const ::std::string & name,
  const ::std::string & symbol,
  ::sstbx::utility::Key<T> & propertyKey,
  const ::std::string & defaultFormatString
)
{
  return ::std::auto_ptr<InfoToken>(new StructurePropertyToken<T>(name, symbol, propertyKey, defaultFormatString));
}

template <typename T, typename Getter>
::std::auto_ptr<InfoToken>
makeFunctionToken(
  const ::std::string & name,
  const ::std::string & symbol,
  Getter getter,
  const ::std::string & defaultFormatString
)
{
  return ::std::auto_ptr<InfoToken>(new FunctionToken<T, Getter>(name, symbol, getter, defaultFormatString));
}

}
}

#endif /* INFO_TOKEN_DETAIL_H */
