/*
 * ProgramOptions.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "yaml/ProgramOptions.h"

#ifdef SSLIB_USE_YAML

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace yaml {

bool insertScalar(YAML::Node & node, const ::std::string & pathValueString)
{
  ::std::vector< ::std::string> pathValuePair;
  boost::split(pathValuePair, pathValueString, ::boost::is_any_of("="));
  if(pathValuePair.size() != 2)
    return false;
  return insertScalar(node, pathValuePair[0], pathValuePair[1]);
}

bool insertScalar(YAML::Node & node, const ::std::string & path, const ::std::string & value)
{
  ::std::vector< ::std::string> pathEntries;
  boost::split(pathEntries, path, ::boost::is_any_of("."));

  bool foundErrors = false;
  YAML::Node * currentLocation = &node;
  BOOST_FOREACH(const ::std::string & pathEntry, pathEntries)
  {
    if(!pathEntry.empty())
    {
      currentLocation = &(*currentLocation)[pathEntry];
    }
    else
      foundErrors = true;
  }
  *currentLocation = value;

  return !foundErrors;
}

bool insertScalars(YAML::Node & node, const ::std::vector< ::std::string> & values)
{
  ::std::vector< ::std::string> pathValuePair;
  
  bool foundErrors = false;
  BOOST_FOREACH(const ::std::string & pathValueString, values)
  {
    boost::split(pathValuePair, pathValueString, ::boost::is_any_of("="));
    if(pathValuePair.size() == 2)
      foundErrors |= insertScalar(node, pathValuePair[0], pathValuePair[1]);
    else
      foundErrors = true;
  }

  return !foundErrors;
}

}
}

#endif /* SSLIB_USE_YAML */

