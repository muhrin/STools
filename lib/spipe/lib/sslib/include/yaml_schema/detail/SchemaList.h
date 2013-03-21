/*
 * SchemaList.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_LIST_DETAIL_H
#define SCHEMA_LIST_DETAIL_H

// INCLUDES /////////////////////////////////////////////
#include <sstream>

#include <boost/foreach.hpp>

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace yaml_schema {

template <class EntrySchema>
SchemaList<EntrySchema>::SchemaList():
myLength(-1)
{}

template <class EntrySchema>
SchemaList<EntrySchema>::SchemaList(const EntrySchema & entrySchema):
myEntrySchema(entrySchema),
myLength(-1)
{}

template <class EntrySchema>
SchemaList<EntrySchema>::SchemaList(const SchemaList & toCopy):
myEntrySchema(toCopy.myEntrySchema),
myLength(toCopy.myLength)
{}

template <class EntrySchema>
bool SchemaList<EntrySchema>::valueToNode(
  YAML::Node & node,
  const BindingType & list,
  const bool useDefaultOnFail) const
{
  BOOST_FOREACH(typename BindingType::const_reference value, list)
  {
    YAML::Node listEntry;
    if(myEntrySchema.valueToNode(listEntry, value, useDefaultOnFail))
      node.push_back(listEntry);
  }
  return true;
}

template <class EntrySchema>
bool SchemaList<EntrySchema>::nodeToValue(
  SchemaParse & parse,
  BindingType & list,
  const YAML::Node & node,
  const bool useDefaultOnFail) const
{
  if(!node.IsSequence())
  {
    parse.logError(SchemaParseErrorCode::NODE_TYPE_WRONG, "Expected sequence.");
    return false;
  }

  for(size_t i = 0; i < node.size(); ++i)
  {
    ::std::stringstream ss;
    ss << "[" << i << "]";
    parse.pushPath(ss.str());

    ListEntryType entryValue;
    if(myEntrySchema.nodeToValue(parse, entryValue, node[i], useDefaultOnFail))
      list.push_back(entryValue);

    parse.popPath();
  }
  return true;
}

template <class EntrySchema>
SchemaList<EntrySchema> * SchemaList<EntrySchema>::length(const int length)
{
  myLength = length;
  return this;
}

template <class EntrySchema>
SchemaList<EntrySchema> * SchemaList<EntrySchema>::clone() const
{
  return new SchemaList<EntrySchema>(*this);
}


}
}

#endif /* SCHEMA_LIST_DETAIL_H */
