/*
 * SchemaMap.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_MAP_DETAIL_H
#define SCHEMA_MAP_DETAIL_H

// INCLUDES /////////////////////////////////////////////
#include <boost/foreach.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/type_traits/is_fundamental.hpp>

#include "yaml_schema/SchemaScalar.h"

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace yaml_schema {
namespace detail {

template <typename T, class IsCppFundamental = ::boost::is_fundamental<T> >
struct SchemaEntryAutoGenerator
{
  static SchemaElementBase<T> * generate()
  {
    // If we know nothing else, assume it's a scalar.
    // See ScalarMap.cpp for specialisations
    return new SchemaScalar<T>();
  }
};

class SchemaHeteroMapEntryBase
{
public:
  explicit SchemaHeteroMapEntryBase(const ::std::string & name);
  SchemaHeteroMapEntryBase(const SchemaHeteroMapEntryBase & toCopy);
  virtual ~SchemaHeteroMapEntryBase() {}

  SchemaHeteroMapEntryBase * required();
  bool isRequired() const;

  virtual bool valueToNode(
    YAML::Node & node,
    const  utility::HeterogeneousMap & map,
    const bool useDefaultOnFail
  ) const = 0;
  virtual bool nodeToValue(
    SchemaParse & parse,
    utility::HeterogeneousMap & map,
    const YAML::Node & node,
    const bool useDefaultOnFail
  ) const = 0;
  virtual bool defaultValueToMap(utility::HeterogeneousMap & map) const = 0;

  virtual SchemaHeteroMapEntryBase * clone() const = 0;

  const ::std::string & getName() const;

private:
  bool myRequired;
  const ::std::string myName;
};

template <typename T>
class SchemaHeteroMapEntry : public SchemaHeteroMapEntryBase
{
public:
  SchemaHeteroMapEntry(
    const ::std::string & name,
    const utility::Key<T> & key,
    SchemaElementBase<T> * const element
  );

  SchemaHeteroMapEntry(const SchemaHeteroMapEntry & toCopy);

  SchemaElementBase<T> * element();

  // From SchemaElementBase ///////////////
  virtual bool valueToNode(
    YAML::Node & node,
    const  utility::HeterogeneousMap & map,
    const bool useDefaultOnFail
  ) const;
  virtual bool nodeToValue(
    SchemaParse & parse,
    utility::HeterogeneousMap & map,
    const YAML::Node & node,
    const bool useDefaultOnFail
  ) const;
  virtual bool defaultValueToMap(utility::HeterogeneousMap & map) const;
  virtual SchemaHeteroMapEntry * clone() const;
  // End from SchemaElementBase /////////////

private:
  mutable utility::Key<T> myKey;
  ::boost::scoped_ptr<SchemaElementBase<T> > myElement;
};

template <typename T>
SchemaHeteroMapEntry<T>::SchemaHeteroMapEntry(
  const ::std::string & name,
  const utility::Key<T> & key,
  SchemaElementBase<T> * const element
):
SchemaHeteroMapEntryBase(name),
myKey(key),
myElement(element)
{}

template <typename T>
SchemaHeteroMapEntry<T>::SchemaHeteroMapEntry(const SchemaHeteroMapEntry<T> & toCopy):
SchemaHeteroMapEntryBase(toCopy),
myKey(toCopy.myKey),
myElement(toCopy.myElement->clone())
{}

template <typename T>
SchemaElementBase<T> * SchemaHeteroMapEntry<T>::element()
{
  return myElement.get();
}

template <typename T>
bool SchemaHeteroMapEntry<T>::valueToNode(
  YAML::Node & node,
  const  utility::HeterogeneousMap & map,
  const bool useDefaultOnFail
) const
{
  const T * const value = map.find(myKey);
  if(!value)
    return false;

  return myElement->valueToNode(node, *value, useDefaultOnFail);
}

template <typename T>
bool SchemaHeteroMapEntry<T>::nodeToValue(
  SchemaParse & parse,
  utility::HeterogeneousMap & map,
  const YAML::Node & node,
  const bool useDefaultOnFail
) const
{
  SchemaParse::PathPusher pusher(parse, getName());

  if(!node.IsDefined())
  {
    // Check for a default
    if(myElement->getDefault())
    {
      map.insert(myKey, *myElement->getDefault());
      return true;
    }
    else if(isRequired())
      parse.logError(SchemaParseErrorCode::REQUIRED_VALUE_MISSING, "Required map entry missing.");
    return false;
  }

  T value;
  const bool succeeded = myElement->nodeToValue(parse, value, node, useDefaultOnFail);
  
  if(succeeded)
    map.insert(myKey, value);

  return succeeded;
}

template <typename T>
bool SchemaHeteroMapEntry<T>::defaultValueToMap(utility::HeterogeneousMap & map) const
{
  if(!myElement->getDefault())
    return false;

  map.insert(myKey, *myElement->getDefault());
  return true;
}

template <typename T>
SchemaHeteroMapEntry<T> *
SchemaHeteroMapEntry<T>::clone() const
{
  return new SchemaHeteroMapEntry(*this);
}

} // namespace detail

template <typename T>
bool SchemaHomoMap<T>::valueToNode(YAML::Node & node, const BindingType & value, const bool useDefaultOnFail) const
{
  typename EntriesMap::const_iterator it;
  BOOST_FOREACH(typename BindingType::const_reference entry, value)
  {
    it = myEntries.find(entry.first);
    if(it != myEntries.end())
    {
      YAML::Node entryNode;
      it.second.valueToNode(entryNode, entry.second, useDefaultOnFail);
      node[entry.first] = entryNode;
    }
  }
}

template <typename T>
bool SchemaHomoMap<T>::nodeToValue(SchemaParse & parse, BindingType & value, const YAML::Node & node, const bool useDefaultOnFail) const
{
  if(!node.IsMap())
  {
    parse.logError(SchemaParseErrorCode::NODE_TYPE_WRONG, "Expected map");
    return false;
  }

  // TODO: Fix this, this method is wrong.  It needs to be Node that is processed, not value!
  typename EntriesMap::const_iterator it;
  BOOST_FOREACH(typename BindingType::const_reference entry, value)
  {
    if(node[entry.first])
    {
      T value;
      entry.second->nodeToValue(parse, value, node[entry.first], useDefaultOnFail);
    }
  }
}

template <typename T>
void SchemaHomoMap<T>::addEntry(const ::std::string & name, const detail::SchemaElementBase<T> * const element)
{
  myEntries.insert(name, element);
}

template <typename T>
detail::SchemaHeteroMapEntry<T> * SchemaHeteroMap::addEntry(
  const ::std::string & name,
  const utility::Key<T> & key,
  detail::SchemaElementBase<T> * const element
)
{
  const utility::KeyId * keyId = key.getId();
  detail::SchemaHeteroMapEntry<T> * const entry = new detail::SchemaHeteroMapEntry<T>(name, key, element);
  myEntries.insert(keyId, entry);
  return entry;
}

template <typename T>
detail::SchemaHeteroMapEntry<T> * SchemaHeteroMap::addScalarEntry(
  const ::std::string & name,
  const utility::Key<T> & key
)
{
  return addEntry(name, key, detail::SchemaEntryAutoGenerator<T>::generate());
}

} // namespace yaml_schema
} // namespace sstbx

#endif /* SCHEMA_MAP_DETAIL_H */
