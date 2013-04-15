/*
 * OptionsParsing.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "input/OptionsParsing.h"

#include <iostream>

#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>

#include <math/Random.h>
#include <utility/HeterogeneousMap.h>

#include <factory/MapEntries.h>

// From SSLib
#include <yaml/ProgramOptions.h>

// NAMESPACES ////////////////////////////////

namespace stools {
namespace input {

int parseYaml(YAML::Node & nodeOut, const ::std::string & inputFile)
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

bool insertScalarValues(YAML::Node & node, const ::std::vector< ::std::string> & scalarValues)
{
  BOOST_FOREACH(const ::std::string & keyValueString, scalarValues)
  {
    if(!sstbx::yaml::insertScalar(node, keyValueString))
    {
      std::cerr << "Invalid input: " << keyValueString;
      return false;
    }
  }
  return true;
}

void seedRandomNumberGenerator(const ::sstbx::utility::HeterogeneousMap & options)
{
  namespace spf = ::spipe::factory;
  namespace ssm = ::sstbx::math;

  const ::std::string * const rngSeed = options.find(spf::RNG_SEED);
  if(rngSeed)
  {
    // Is it an integer?
    bool userSuppliedSeed = false;
    try
    {
      ssm::seed(::boost::lexical_cast<unsigned int>(*rngSeed));
      userSuppliedSeed = true;
    }
    catch(const ::boost::bad_lexical_cast & /*e*/)
    {}
    if(!userSuppliedSeed && rngSeed->find("time"))
      ssm::seed();
  }
}

} // namespace stools
} // namespace input
