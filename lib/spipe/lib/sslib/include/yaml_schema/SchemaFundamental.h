/*
 * SchemaFundamental.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_FUNDAMENTAL_H
#define SCHEMA_FUNDAMENTAL_H

// INCLUDES /////////////////////////////////////////////
#include <boost/optional.hpp>

#include "yaml_schema/detail/SchemaElement.h"

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////


namespace sstbx {
namespace yaml_schema {

template <typename T>
class SchemaFundamental : public detail::SchemaElementBase<T>
{
public:
  typedef T BindingType;

  // NOTE: Have to pull in any methods that we want to use directly (i.e. without this->)
  // here because of the way identifier lookup works with template base classes.
  // See: e.g. http://stackoverflow.com/questions/5286922/g-template-parameter-error
  using detail::SchemaElementBase<T>::getDefault;
  using detail::SchemaElementBase<T>::isRequired;

  virtual bool valueToNode(YAML::Node & node, const T & value, const bool useDefaultOnFail) const;
  virtual bool nodeToValue(SchemaParse & parse, T & value, const YAML::Node & node, const bool useDefaultOnFail) const;
  virtual SchemaFundamental * clone() const;
};

}
}

#include "yaml_schema/detail/SchemaFundamental.h"

#endif /* SCHEMA_FUNDAMENTAL_H */
