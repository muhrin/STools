/*
 * TranscodeFactory.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/yaml/TranscodeFactory.h"

#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include "spl/io/Parsing.h"
#include "spl/yaml/TranscodeGeneral.h"

// NAMESPACES ////////////////////////////////

namespace ssf = ::spl::factory;

namespace YAML {

Node
convert< ::spl::factory::MinMax>::encode(const ::spl::factory::MinMax & rhs)
{
  namespace kw = ssf::sslib_yaml_keywords;

  const ::std::string min(kw::MIN);
  const ::std::string max(kw::MAX);
  Node node;
  if(rhs.min)
    node[kw::MIN] = *(rhs.min);
  if(rhs.max)
    node[kw::MAX] = *(rhs.max);
  return node;
}

bool
convert< ::spl::factory::MinMax>::decode(const Node & node,
    ::spl::factory::MinMax & rhs)
{
  namespace kw = ssf::sslib_yaml_keywords;

  if(!node.IsMap())
    return false;

  try
  {
    if(node[kw::MIN])
      rhs.min.reset(node[kw::MIN].as< double>());
    if(node[kw::MAX])
      rhs.max.reset(node[kw::MAX].as< double>());
  }
  catch(const YAML::Exception & /*e*/)
  {
    return false;
  }
  return true;
}

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
  namespace ssbc = ::spl::build_cell;
  namespace ssio = ::spl::io;

  if(!node.IsScalar())
    return false;

  std::stringstream ss(node.Scalar());
  ss >> rhs;

  return true;
}

}

