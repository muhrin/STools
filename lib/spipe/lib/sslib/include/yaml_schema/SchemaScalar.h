/*
 * SchemaScalar.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_SCALAR_H
#define SCHEMA_SCALAR_H

// INCLUDES /////////////////////////////////////////////
#include <boost/optional.hpp>

#include "yaml_schema/detail/SchemaElement.h"

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////


namespace sstbx {
namespace yaml_schema {

template <typename T>
class SchemaScalar : public detail::SchemaElementBase<T>
{
public:
  typedef T BindingType;

  virtual bool valueToNode(YAML::Node & node, const T & value, const bool useDefaultOnFail) const;
  virtual bool nodeToValue(SchemaParse & parse, T & value, const YAML::Node & node, const bool useDefaultOnFail) const;
  virtual SchemaScalar * clone() const;
};


template <typename T>
bool SchemaScalar<T>::valueToNode(YAML::Node & node, const T & value, const bool useDefaultOnFail) const
{
  // TODO: Find out how to check if transcoding failed
  node = value;
  return true;
}

template <typename T>
bool SchemaScalar<T>::nodeToValue(SchemaParse & parse, T & value, const YAML::Node & node, const bool useDefaultOnFail) const
{
  if(!node.IsDefined())
  {
    if(isRequired())
      parse.logError(SchemaParseErrorCode::REQUIRED_VALUE_MISSING, "Required scalar value missing.");
    return false;
  }
  // TODO: Eventually reinstate this conditional
  //if(!node.IsScalar())
  //{
  //  parse.logError(SchemaParseErrorCode::NODE_TYPE_WRONG, "Expected scalar.");
  //  return false;
  //}

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
    parse.logError(SchemaParseErrorCode::TYPE_CONVERSION_FAILED, "Value is not a valid scalar of the right type.");
    return false;
  }
  return true;
}

template <typename T>
SchemaScalar<T> * SchemaScalar<T>::clone() const
{
  return new SchemaScalar(*this);
}

}
}

#endif /* SCHEMA_SCALAR_H */
