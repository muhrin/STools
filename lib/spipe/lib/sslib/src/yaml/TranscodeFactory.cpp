/*
 * TranscodeFactory.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "yaml/TranscodeFactory.h"

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/tokenizer.hpp>

#include "factory/SsLibYamlKeywords.h"
#include "io/Parsing.h"
#include "yaml/TranscodeGeneral.h"

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
  
  // Don't bother printing if it's a null span and the count is 1
  if(!(rhs.count.nullSpan() && rhs.count.lower() == 1))
  {
    Node range;
    range = rhs.count; // Use existing range transcoding
    ss << range.Scalar();
  }

  node = ss.str();

  return node;
}

bool convert< ::sstbx::factory::AtomSpeciesCount>::decode(
  const Node & node, ::sstbx::factory::AtomSpeciesCount & rhs)
{
  namespace ssbc = ::sstbx::build_cell;
  namespace ssio = ::sstbx::io;

  if(!node.IsScalar())
    return false;

  static const ::boost::regex RE_SPECIES_COUNT("([[:alpha:]]+)[[:blank:]]*(" + ssio::PATTERN_RANGE + ")");

  const ::std::string speciesString = node.Scalar();


  ::boost::smatch match;
  if(::boost::regex_search(speciesString, match, RE_SPECIES_COUNT))
  {
    rhs.species.assign(match[1].first, match[1].second);
    rhs.count.set(1, 1);
    if(match.size() > 2)
    {
      // We maybe have a count range, so make a node and use the
      // current transcoding to parse that
      const ::std::string count(match[2].first, match[2].second);
      Node countNode;
      countNode = count;
      try
      {
        rhs.count = countNode.as<ssbc::AtomsDescription::CountRange>();
      }
      catch(const YAML::Exception & /*e*/)
      {
        return false;
      }
    }
    return true;
  }
  return false;
}

}


