/*
 * SchemaList.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_LIST_H
#define SCHEMA_LIST_H

// INCLUDES /////////////////////////////////////////////

#include <vector>

#include "yaml_schema/detail/SchemaElement.h"

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace yaml_schema {

template <class EntrySchema>
class SchemaList :
  public detail::SchemaElementBase< ::std::vector<typename EntrySchema::BindingType> >
{
  typedef typename EntrySchema::BindingType ListEntryType;
public:
  typedef ::std::vector<ListEntryType> BindingType;

  SchemaList(); 
  SchemaList(const EntrySchema & entrySchema);
  SchemaList(const SchemaList & toCopy);
  virtual ~SchemaList() {}

  virtual bool valueToNode(YAML::Node & node, const BindingType & list, const bool useDefaultOnFail) const;
  virtual bool nodeToValue(SchemaParse & parse, BindingType & list, const YAML::Node & node, const bool useDefaultOnFail) const;

  SchemaList * length(const int length);

  virtual SchemaList * clone() const;

private:
  int myLength;
  const EntrySchema myEntrySchema;
};


}
}

#include "yaml_schema/detail/SchemaList.h"

#endif /* SCHEMA_LIST_H */
