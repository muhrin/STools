/*
 * YamlInputObjectAdapter.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"

#ifdef SP_USE_YAML
#include "common/YamlInputObjectAdapter.h"

#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>

#include "common/YamlInputObjectIteratorAdapter.h"
 

// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace common
{

using ::sstbx::factory::IInputObject;

YamlInputObjectAdapter::YamlInputObjectAdapter(const YAML::Node & node, const ::std::string & key):
myNode(node),
myKey(key)
{}

IInputObject::Type YamlInputObjectAdapter::getType() const
{
  IInputObject::Type type = IInputObject::UNDEFINED;

  switch(myNode.Type())
  {
  case YAML::NodeType::Scalar:
    type = IInputObject::SCALAR;
    break;
  case YAML::NodeType::Map:
    type = IInputObject::MAP;
    break;
  case YAML::NodeType::Sequence:
    type = IInputObject::LIST;
    break;
  }

  return type;
}


size_t YamlInputObjectAdapter::getSize() const
{
  return myNode.size();
}

const ::std::string & YamlInputObjectAdapter::getValue() const
{
  return myNode.Scalar();
}

const ::std::string & YamlInputObjectAdapter::getKey() const
{
  return myKey;
}

const YamlInputObjectAdapter::InterfacePtrConst
YamlInputObjectAdapter::operator [](const size_t idx) const
{
  //if(getType() == IInputObject::MAP)
  //{
  //  return this->operator[](::boost::lexical_cast<::std::string>(idx));   
  //}

  InterfacePtrConst childPtr;
  if(getType() == IInputObject::LIST)
  {
    childPtr.reset(new YamlInputObjectAdapter(myNode[idx], ""));
  }
  return childPtr;
}

const YamlInputObjectAdapter::InterfacePtrConst
YamlInputObjectAdapter::operator [](const ::std::string & key) const
{
  InterfacePtrConst childPtr;
  if(getType() == IInputObject::MAP)
  {
    childPtr.reset(new YamlInputObjectAdapter(myNode[key], key));
  }
  return childPtr;
}

YamlInputObjectAdapter::const_iterator YamlInputObjectAdapter::begin() const
{
  typedef ::boost::shared_ptr<YamlInputObjectIteratorAdapter> IterSharedPtr;
  return const_iterator(IterSharedPtr(new YamlInputObjectIteratorAdapter(myNode.begin(), myNode.IsMap())));
}

YamlInputObjectAdapter::const_iterator YamlInputObjectAdapter::end() const
{
  typedef ::boost::shared_ptr<YamlInputObjectIteratorAdapter> IterSharedPtr;
  return const_iterator(IterSharedPtr(new YamlInputObjectIteratorAdapter(myNode.end(), myNode.IsMap())));
}

}
}

#endif // SP_USE_YAML
