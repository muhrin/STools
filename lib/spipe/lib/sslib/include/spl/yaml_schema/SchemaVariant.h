/*
 * SchemaVariant.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_VARIANT_H
#define SCHEMA_VARIANT_H

// INCLUDES /////////////////////////////////////////////
#include <boost/variant.hpp>

#include "spl/yaml_schema/detail/SchemaElement.h"

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace yaml_schema {

template <class ScalarSchema, class ListSchema, class MapSchema>
class SchemaScalarListMap :
  public detail::SchemaElementBase<
    ::boost::variant<
      typename ScalarSchema::BindingType,
      typename ListSchema::BindingType,
      typename MapSchema::BindingType>
  >
{
public:
  typedef ::boost::variant<
    typename ScalarSchema::BindingType,
    typename ListSchema::BindingType,
    typename MapSchema::BindingType
  > BindingType;

  SchemaScalarListMap(
    const ScalarSchema & scalarSchema,
    const ListSchema & listSchema,
    const MapSchema & mapSchema);
  SchemaScalarListMap(const SchemaScalarListMap & toCopy);

  // NOTE: Have to pull in any methods that we want to use directly (i.e. without this->)
  // here because of the way identifier lookup works with template base classes.
  // See: e.g. http://stackoverflow.com/questions/5286922/g-template-parameter-error
  using detail::SchemaElementBase<BindingType>::getDefault;
  using detail::SchemaElementBase<BindingType>::isRequired;

  virtual bool valueToNode(YAML::Node & node, const BindingType & binding, const bool useDefaultOnFail) const;
  virtual bool nodeToValue(SchemaParse & parse, BindingType & binding, const YAML::Node & node, const bool useDefaultOnFail) const;

  virtual SchemaScalarListMap * clone() const;

private:
  const ScalarSchema myScalarSchema;
  const ListSchema myListSchema;
  const MapSchema myMapSchema;
};

template <class ListSchema, class MapSchema>
class SchemaListMap :
  public detail::SchemaElementBase<
    ::boost::variant<
      typename ListSchema::BindingType,
      typename MapSchema::BindingType>
  >
{
public:
  typedef ::boost::variant<
    typename ListSchema::BindingType,
    typename MapSchema::BindingType
  > BindingType;

  SchemaListMap();
  SchemaListMap(
    const ListSchema & listSchema,
    const MapSchema & mapSchema);
  SchemaListMap(const ListSchema & listSchema);
  SchemaListMap(const MapSchema & mapSchema);
  SchemaListMap(const SchemaListMap & toCopy);

  // NOTE: Have to pull in any methods that we want to use directly (i.e. without this->)
  // here because of the way identifier lookup works with template base classes.
  // See: e.g. http://stackoverflow.com/questions/5286922/g-template-parameter-error
  using detail::SchemaElementBase<BindingType>::getDefault;
  using detail::SchemaElementBase<BindingType>::isRequired;

  virtual bool valueToNode(YAML::Node & node, const BindingType & binding, const bool useDefaultOnFail) const;
  virtual bool nodeToValue(SchemaParse & parse, BindingType & binding, const YAML::Node & node, const bool useDefaultOnFail) const;

  virtual SchemaListMap * clone() const;

private:
  const ListSchema myListSchema;
  const MapSchema myMapSchema;
};


}
}

#include "spl/yaml_schema/detail/SchemaVariant.h"

#endif /* SCHEMA_VARIANT_H */
