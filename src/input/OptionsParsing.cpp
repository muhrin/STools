/*
 * OptionsParsing.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "input/OptionsParsing.h"

#include <ctime>
#include <iostream>

#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>

#include <spl/math/Random.h>
#include <spl/utility/HeterogeneousMap.h>
#include <spl/os/Process.h>
#include <spl/yaml/ProgramOptions.h>

#include <factory/MapEntries.h>

// NAMESPACES ////////////////////////////////

namespace stools {
namespace input {

int
parseYaml(YAML::Node & nodeOut, const ::std::string & inputFile)
{
  try
  {
    nodeOut = YAML::LoadFile(inputFile);
  }
  catch(const YAML::Exception & e)
  {
    ::std::cout << e.what();
    return 1;
  }
  return 0;
}

bool
insertScalarValues(YAML::Node & node,
    const ::std::vector< ::std::string> & scalarValues)
{
  BOOST_FOREACH(const ::std::string & keyValueString, scalarValues)
  {
    if(!spl::yaml::insertScalar(node, keyValueString))
    {
      std::cerr << "Invalid input: " << keyValueString;
      return false;
    }
  }
  return true;
}

void
seedRandomNumberGenerator(const ::std::string & seed)
{
  namespace spf = ::spipe::factory;
  namespace ssm = ::spl::math;

  if(seed == "time")
    ssm::seed(
        static_cast< unsigned int>(time(NULL))
            * static_cast< unsigned int>(::spl::os::getProcessId()));
  else
  {
    try
    {
      ssm::seed(::boost::lexical_cast< unsigned int>(seed));
    }
    catch(const ::boost::bad_lexical_cast & /*e*/)
    {
    }
  }

}

} // namespace stools
} // namespace input
