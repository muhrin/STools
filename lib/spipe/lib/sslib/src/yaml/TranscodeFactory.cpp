/*
 * TranscodeFactory.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/yaml/TranscodeFactory.h"


// NAMESPACES ////////////////////////////////

namespace ssf = ::spl::factory;

namespace YAML {

// factory::AtomSpeciesCount
Node
convert< ::spl::factory::AtomSpeciesCount>::encode(
    const ::spl::factory::AtomSpeciesCount & rhs)
{
  Node node;

  ::std::stringstream ss;
  ss << rhs;
  node = ss.str();

  return node;
}

bool
convert< ::spl::factory::AtomSpeciesCount>::decode(const Node & node,
    ::spl::factory::AtomSpeciesCount & rhs)
{
  if(!node.IsScalar())
    return false;

  std::stringstream ss(node.Scalar());
  ss >> rhs;

  return true;
}

}

