/*
 * YamlInputObjectAdapter.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef YAML_INPUT_OBJECT_ADAPTER_H
#define YAML_INPUT_OBJECT_ADAPTER_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#ifdef SP_USE_YAML

#include <yaml-cpp/yaml.h>

#include <factory/IInputObject.h>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe
{
namespace common
{

class YamlInputObjectAdapter : public ::sstbx::factory::IInputObject
{
public:

  typedef ::sstbx::factory::IInputObject::SharedPtrConstTyp  InterfacePtrConst;
  typedef ::sstbx::factory::IInputObject::const_iterator     const_iterator;

  YamlInputObjectAdapter(const YAML::Node & node, const ::std::string & key);

  virtual Type                    getType() const;
  virtual size_t                  getSize() const;
  virtual const ::std::string &   getValue() const;
  virtual const ::std::string &   getKey() const;

  // Access mechanism for sequences
  virtual const SharedPtrConstTyp  operator [](const size_t idx) const;

  // Access mechanism for maps
  virtual const SharedPtrConstTyp  operator [](const ::std::string & key) const;

  virtual const_iterator begin() const;
  virtual const_iterator end() const;

private:

  const YAML::Node      myNode;
  const ::std::string   myKey;
};


}
}

#endif /* SP_USE_YAML */
#endif /* YAML_INPUT_OBJECT_ADAPTER_H */
