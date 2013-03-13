/*
 * TranscodeFactory.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "yaml/TranscodeFactory.h"

#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include "factory/SsLibYamlKeywords.h"

// NAMESPACES ////////////////////////////////

namespace ssf = ::sstbx::factory;

namespace YAML {

Node convert< ::sstbx::factory::MinMax>::encode(const ::sstbx::factory::MinMax & rhs)
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

bool convert< ::sstbx::factory::MinMax>::decode(const Node & node, ::sstbx::factory::MinMax & rhs)
{
  namespace kw = ssf::sslib_yaml_keywords;

  if(!node.IsMap())
    return false;

  try
  {
    if(node[kw::MIN])
      rhs.min.reset(node[kw::MIN].as<double>());
    if(node[kw::MAX])
      rhs.max.reset(node[kw::MAX].as<double>());
  }
  catch(const YAML::Exception & /*e*/)
  {
    return false;
  }
  return true;
}

// factory::AtomSpeciesCount
Node convert< ::sstbx::factory::AtomSpeciesCount>::encode(const ::sstbx::factory::AtomSpeciesCount & rhs)
{
  Node node;
  ::std::stringstream ss;
  ss << rhs.species;
  
  // Don't bother printing if only 1
  if(rhs.count != 1)
  {
    ss << " " << rhs.count;
  }
  node = ss.str();

  return node;
}

bool convert< ::sstbx::factory::AtomSpeciesCount>::decode(
  const Node & node, ::sstbx::factory::AtomSpeciesCount & rhs)
{
  typedef boost::tokenizer<boost::char_separator<char> > Tok;
  const boost::char_separator<char> tokSep(" \t");

  const ::std::string species(node.as<::std::string>());
  const Tok tok(species, tokSep);

  Tok::const_iterator it = tok.begin();

  if(it != tok.end())
  {
    rhs.species = *it;

    if(++it != tok.end())
    {
      try
      {
        rhs.count = ::boost::lexical_cast<unsigned int>(*it);
      }
      catch(::boost::bad_lexical_cast)
      {
        return false;
      }
    }
  }
  else
    return false;

  return true;
}

}


