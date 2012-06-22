/*
 * YamlInputObjectIteratorAdapter.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"

#ifdef SP_USE_YAML
#include "common/YamlInputObjectIteratorAdapter.h"

#include "common/YamlInputObjectAdapter.h"

// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace common
{

YamlInputObjectIteratorAdapter::YamlInputObjectIteratorAdapter(
  const YAML::const_iterator & iter, const bool isMap):
myIter(iter),
myIsMap(isMap)
{}

YamlInputObjectIteratorAdapter::Base &
YamlInputObjectIteratorAdapter::operator++()
{
  ++myIter;
  return *this;
}

YamlInputObjectIteratorAdapter::Reference
YamlInputObjectIteratorAdapter::operator*()
{
  if(myIsMap)
  {
    return Value(new YamlInputObjectAdapter(myIter->second, myIter->first.as<std::string>()));
  }
  else
  {
    return Value(new YamlInputObjectAdapter(*myIter, ""));
  }
}

bool YamlInputObjectIteratorAdapter::operator==(const YamlInputObjectIteratorAdapter::Base & rhs) const
{
  // Nastyness: have to use RTTI
  const YamlInputObjectIteratorAdapter * const rhsAdapter =
    dynamic_cast<const YamlInputObjectIteratorAdapter * const>(&rhs);

  // Are we the same type?
  if(!rhsAdapter)
    return false;

  // Compare out iterators
  return myIter == rhsAdapter->myIter;
}

::boost::shared_ptr<YamlInputObjectIteratorAdapter::Base> YamlInputObjectIteratorAdapter::clone()
{
  return ::boost::shared_ptr<Base>(new YamlInputObjectIteratorAdapter(myIter, myIsMap));
}

}
}

#endif // SP_USE_YAML
