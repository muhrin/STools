/*
 * YamlInputObjectIteratorAdapter.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef YAML_INPUT_OBJECT_ITERATOR_ADAPTER_H
#define YAML_INPUT_OBJECT_ITERATOR_ADAPTER_H

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

class YamlInputObjectIteratorAdapter : public
  ::sstbx::factory::IInputObject::IteratorBase<
    ::sstbx::factory::IInputObject::SharedPtrConstTyp,
    ::sstbx::factory::IInputObject::SharedPtrConstTyp
  >
{
public:

  typedef ::sstbx::factory::IInputObject::IteratorBase<
    ::sstbx::factory::IInputObject::SharedPtrConstTyp,
    ::sstbx::factory::IInputObject::SharedPtrConstTyp
  > Base;
  typedef Base::Value       Value;
  typedef Base::Reference   Reference;

  YamlInputObjectIteratorAdapter(const YAML::const_iterator & iter, const bool isMap);

  virtual Base & operator++();
  virtual Reference operator*();
  virtual bool operator==(const IteratorBase & rhs) const;
  virtual ::boost::shared_ptr<Base> clone();

private:
  YAML::const_iterator    myIter;
  const bool              myIsMap;

};


}
}

#endif /* SP_USE_YAML */
#endif /* YAML_INPUT_OBJECT_ITERATOR_ADAPTER_H */
