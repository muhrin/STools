/*
 * SchemaWrapper.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_WRAPPER_H
#define SCHEMA_WRAPPER_H

// INCLUDES /////////////////////////////////////////////
#include "yaml_schema/detail/SchemaElement.h"

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////


namespace sstbx {
namespace yaml_schema {

template <typename Wrapper>
class SchemaWrapper : public detail::SchemaElementBase<typename Wrapper::ValueType>
{
public:
  typedef typename Wrapper::ValueType BindingType;

  virtual bool valueToNode(
    YAML::Node & node,
    const BindingType & value,
    const bool useDefaultOnFail
  ) const;
  virtual bool nodeToValue(
    SchemaParse & parse,
    BindingType & value,
    const YAML::Node & node,
    const bool useDefaultOnFail
  ) const;
  virtual SchemaWrapper * clone() const;
};

}
}

#include "yaml_schema/detail/SchemaWrapper.h"

#endif /* SCHEMA_WRAPPER_H */
