/*
 * SchemaVariant.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_VARIANT_DETAIL_H
#define SCHEMA_VARIANT_DETAIL_H

// INCLUDES /////////////////////////////////////////////

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace yaml_schema {

struct _null {typedef void * BindingType;};

template <class ScalarSchema, class ListSchema, class MapSchema>
class ValueToNodeVisitor : public boost::static_visitor<bool>
{
public:
  ValueToNodeVisitor(
      YAML::Node & node,
      const bool useDefaultOnFail,
      const ScalarSchema & scalar,
      const ListSchema & list,
      const MapSchema & map
    ):
    myNode(node),
    myUseDefaultOnFail(useDefaultOnFail),
    myScalar(scalar),
    myList(list),
    myMap(map)
  {}

  bool operator()(const typename ScalarSchema::BindingType & binding) const
  { return myScalar.valueToNode(myNode, binding, myUseDefaultOnFail); }

  bool operator()(const typename ListSchema::BindingType & binding) const
  { return myList.valueToNode(myNode, binding, myUseDefaultOnFail); }

  bool operator()(const typename MapSchema::BindingType & binding) const
  { return myMap.valueToNode(myNode, binding, myUseDefaultOnFail); }

private:
  YAML::Node & myNode;
  const bool myUseDefaultOnFail;
  const ScalarSchema & myScalar;
  const ListSchema & myList;
  const MapSchema & myMap;
};

template <class ScalarSchema, class ListSchema, class MapSchema>
SchemaScalarListMap<ScalarSchema, ListSchema, MapSchema>::SchemaScalarListMap(
  const ScalarSchema & scalarSchema,
  const ListSchema & listSchema,
  const MapSchema & mapSchema):
myScalarSchema(scalarSchema),
myListSchema(listSchema),
myMapSchema(mapSchema)
{}

template <class ScalarSchema, class ListSchema, class MapSchema>
SchemaScalarListMap<ScalarSchema, ListSchema, MapSchema>::SchemaScalarListMap(
  const SchemaScalarListMap & toCopy):
SchemaElementBase(toCopy),
myScalarSchema(toCopy.myScalarSchema),
myListSchema(toCopy.myListSchema),
myMapSchema(toCopy.myMapSchema)
{}

template <class ScalarSchema, class ListSchema, class MapSchema>
bool SchemaScalarListMap<ScalarSchema, ListSchema, MapSchema>::valueToNode(
  YAML::Node & node, const BindingType & binding, const bool useDefaultOnFail) const
{
  ValueToNodeVisitor<ScalarSchema, ListSchema, MapSchema>
    valueToNodeVisitor(node, useDefaultOnFail, myScalarSchema, myListSchema, myMapSchema);

  return ::boost::apply_visitor(valueToNodeVisitor, binding);
}

template <class ScalarSchema, class ListSchema, class MapSchema>
bool SchemaScalarListMap<ScalarSchema, ListSchema, MapSchema>::nodeToValue(
  SchemaParse & parse,
  BindingType & binding,
  const YAML::Node & node,
  const bool useDefaultOnFail) const
{
  if(!node.IsDefined())
  {
    if(isRequired())
      parse.logError(REQUIRED_VALUE_MISSING, "Missing required scalar-list-map");
    return false;
  }

  if(node.IsScalar())
  {
    // Make the variant contain the scalar binding type
    binding = ScalarSchema::BindingType();
    return myListSchema.nodeToValue(
      parse,
      ::boost::get<ScalarSchema::BindingType>(binding),
      node,
      useDefaultOnFail
    );
  }
  else if(node.IsSequence())
  {
    // Make the variant contain the list binding type
    binding = ListSchema::BindingType();
    return myListSchema.nodeToValue(
      parse,
      ::boost::get<ListSchema::BindingType>(binding),
      node,
      useDefaultOnFail
    );
  }
  else if(node.IsMap())
  {
    // Make the variant contain the list binding type
    binding = MapSchema::BindingType();
    return myMapSchema.nodeToValue(
      parse,
      ::boost::get<MapSchema::BindingType>(binding),
      node,
      useDefaultOnFail
    );
  }
}

template <class ScalarSchema, class ListSchema, class MapSchema>
SchemaScalarListMap<ScalarSchema, ListSchema, MapSchema> *
SchemaScalarListMap<ScalarSchema, ListSchema, MapSchema>::clone() const
{
  return new SchemaScalarListMap(*this);
}

template <class ListSchema, class MapSchema>
SchemaListMap<ListSchema, MapSchema>::SchemaListMap()
{}

template <class ListSchema, class MapSchema>
SchemaListMap<ListSchema, MapSchema>::SchemaListMap(
  const ListSchema & listSchema,
  const MapSchema & mapSchema):
myListSchema(listSchema),
myMapSchema(mapSchema)
{}

template <class ListSchema, class MapSchema>
SchemaListMap<ListSchema, MapSchema>::SchemaListMap(const ListSchema & listSchema):
myListSchema(listSchema)
{}

template <class ListSchema, class MapSchema>
SchemaListMap<ListSchema, MapSchema>::SchemaListMap(const MapSchema & mapSchema):
myMapSchema(mapSchema)
{}

template <class ListSchema, class MapSchema>
SchemaListMap<ListSchema, MapSchema>::SchemaListMap(
  const SchemaListMap & toCopy):
SchemaElementBase(toCopy),
myListSchema(toCopy.myListSchema),
myMapSchema(toCopy.myMapSchema)
{}

template <class ListSchema, class MapSchema>
bool SchemaListMap<ListSchema, MapSchema>::valueToNode(
  YAML::Node & node, const BindingType & binding, const bool useDefaultOnFail) const
{
  ValueToNodeVisitor<_null, ListSchema, MapSchema>
    valueToNodeVisitor(node, useDefaultOnFail, _null(), myListSchema, myMapSchema);

  return ::boost::apply_visitor(valueToNodeVisitor, binding);
}

template <class ListSchema, class MapSchema>
bool SchemaListMap<ListSchema, MapSchema>::nodeToValue(
  SchemaParse & parse,
  BindingType & binding,
  const YAML::Node & node,
  const bool useDefaultOnFail
) const
{
  if(!node.IsDefined())
  {
    if(isRequired())
      parse.logError(SchemaParseErrorCode::REQUIRED_VALUE_MISSING, "Missing required list map");
    return false;
  }

  if(node.IsSequence())
  {
    // Make the variant contain the list binding type
    binding = ListSchema::BindingType();
    return myListSchema.nodeToValue(
      parse,
      ::boost::get<ListSchema::BindingType>(binding),
      node,
      useDefaultOnFail
    );
  }
  else if(node.IsMap())
  {
    // Make the variant contain the list binding type
    binding = MapSchema::BindingType();
    return myMapSchema.nodeToValue(
      parse,
      ::boost::get<MapSchema::BindingType>(binding),
      node,
      useDefaultOnFail
    );
  }
  else
    return false;

  return true;
}

template <class ListSchema, class MapSchema>
SchemaListMap<ListSchema, MapSchema> *
SchemaListMap<ListSchema, MapSchema>::clone() const
{
  return new SchemaListMap(*this);
}

}
}

#endif /* SCHEMA_VARIANT_H */
