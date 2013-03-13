/*
 * SchemaFundamental.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_FUNDAMENTAL_DETAIL_H
#define SCHEMA_FUNDAMENTAL_DETAIL_H

// INCLUDES /////////////////////////////////////////////

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////


namespace sstbx {
namespace yaml_schema {

template <typename T>
bool SchemaFundamental<T>::valueToNode(YAML::Node & node, const T & value, const bool useDefaultOnFail) const
{
  // TODO: Find out how to check if transcoding failed
  node = value;
  return true;
}

template <typename T>
bool SchemaFundamental<T>::nodeToValue(SchemaParse & parse, T & value, const YAML::Node & node, const bool useDefaultOnFail) const
{
  if(!node.IsDefined())
    return false;

  try
  {
    // TODO: Check behaviour of as if value is invalid (i.e. does it just use the default?)
    if(useDefaultOnFail && getDefault())
      value = node.as<T>(*getDefault());
    else
      value = node.as<T>();
  }
  catch(const YAML::Exception & /*e*/)
  {
    parse.logError(TYPE_CONVERSION_FAILED, "Failed to convert value.  Probably a typo.");
    return false;
  }
  return true;
}

template <typename T>
SchemaFundamental<T> * SchemaFundamental<T>::clone() const
{
  return new SchemaFundamental(*this);
}

}
}

#endif /* SCHEMA_FUNDAMENTAL_DETAIL_H */
