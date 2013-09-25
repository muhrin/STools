/*
 * SchemaMap.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/yaml_schema/SchemaMap.h"

#include "spl/yaml_schema/SchemaFundamental.h"

#include <boost/foreach.hpp>

// NAMESPACES ////////////////////////////////

namespace spl {
namespace yaml_schema {

bool SchemaHeteroMap::valueToNode(YAML::Node & node, const BindingType & value, const bool useDefaultOnFail) const
{
  BOOST_FOREACH(EntriesMap::const_reference entry, myEntries)
  {
    YAML::Node entryNode;
    if(entry.second->valueToNode(entryNode, value, useDefaultOnFail))
      node[entry.second->getName()] = entryNode;
  }
  return true;
}

bool SchemaHeteroMap::nodeToValue(
  SchemaParse & parse,
  utility::HeterogeneousMap & map,
  const YAML::Node & node,
  const bool useDefaultOnFail
) const
{
  if(node.IsNull())
  {
    // We are a map with no entries, check that we have no required entries with
    // no default
    bool succeeded = true;
    BOOST_FOREACH(EntriesMap::const_reference entry, myEntries)
    {
      if(!entry.second->defaultValueToMap(map) && entry.second->isRequired())
      {
        parse.logError(
          SchemaParseErrorCode::REQUIRED_VALUE_MISSING,
          "Required map entry missing: " + entry.second->getName() + "."
        );
        succeeded = false;
      }
    }
    return succeeded;
  }

  if(!node.IsMap())
  {
    parse.logError(SchemaParseErrorCode::NODE_TYPE_WRONG, "Expected map.");
    return false;
  }

  BOOST_FOREACH(EntriesMap::const_reference entry, myEntries)
  {
    entry.second->nodeToValue(parse, map, node[entry.second->getName()], useDefaultOnFail);
  }
  return true;

}

SchemaHeteroMap * SchemaHeteroMap::clone() const
{
  return new SchemaHeteroMap(*this);
}


namespace detail {

// Disallow heterogeneous maps from being auto generated.  There is no point
// as the user has the specify what keys they want it to contain.
template <>
struct SchemaEntryAutoGenerator<utility::HeterogeneousMap, boost::false_type> {};

// TODO: Deal with lists

SchemaHeteroMapEntryBase::SchemaHeteroMapEntryBase(const ::std::string & name):
myRequired(false),
myName(name)
{}

SchemaHeteroMapEntryBase::SchemaHeteroMapEntryBase(const SchemaHeteroMapEntryBase & toCopy):
myRequired(false),
myName(toCopy.myName)
{}

SchemaHeteroMapEntryBase * SchemaHeteroMapEntryBase::required()
{
  myRequired = true;
  return this;
}

bool SchemaHeteroMapEntryBase::isRequired() const
{
  return myRequired;
}

const ::std::string & SchemaHeteroMapEntryBase::getName() const
{
  return myName;
}

SchemaHeteroMapEntryBase * new_clone(const SchemaHeteroMapEntryBase & entry)
{
  return entry.clone();
}

} // namespace detail
}
}


