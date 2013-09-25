/*
 * SchemaElementBase.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_ELEMENT_BASE_H
#define SCHEMA_ELEMENT_BASE_H

// INCLUDES /////////////////////////////////////////////

#include <boost/optional.hpp>

#include <yaml-cpp/yaml.h>

#include "spl/yaml_schema/SchemaParse.h"

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////


namespace spl {
namespace yaml_schema {
namespace detail {

template <typename T>
class SchemaElementBase
{
public:
  SchemaElementBase();
  SchemaElementBase(const SchemaElementBase & toCopy);
  virtual ~SchemaElementBase() {}

  SchemaElementBase * required();
  bool isRequired() const;

  SchemaElementBase * defaultValue(const T & defaultValue);
  const ::boost::optional<T> getDefault() const;

  virtual bool valueToNode(YAML::Node & node, const T & value, const bool useDefaultOnFail) const = 0;
  virtual bool nodeToValue(SchemaParse & parse, T & value, const YAML::Node & node, const bool useDefaultOnFail) const = 0;
  bool defaultValueToNode(YAML::Node & node) const;

  virtual SchemaElementBase * clone() const = 0;

private:
  ::boost::optional<T> myDefault;
  bool myRequired;
};

template <typename T>
SchemaElementBase<T>::SchemaElementBase():
myRequired(false)
{}

template <typename T>
SchemaElementBase<T>::SchemaElementBase(const SchemaElementBase & toCopy):
myDefault(toCopy.myDefault),
myRequired(toCopy.myRequired)
{}

template <typename T>
SchemaElementBase<T> * SchemaElementBase<T>::required()
{
  myRequired = true;
  return this;
}

template <typename T>
bool SchemaElementBase<T>::isRequired() const
{
  return myRequired;
}

template <typename T>
SchemaElementBase<T> * SchemaElementBase<T>::defaultValue(const T & defaultValue)
{
  myDefault = defaultValue;
  return this;
}

template <typename T>
const ::boost::optional<T> SchemaElementBase<T>::getDefault() const
{
  return myDefault;
}

template <typename T>
bool SchemaElementBase<T>::defaultValueToNode(YAML::Node & node) const
{
  if(!myDefault)
    return false;

  return valueToNode(node, *myDefault, false);
}

}
}
}

#endif /* SCHEMA_ELEMENT_BASE_H */
