/*
 * OptionsParsing.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "input/OptionsParsing.h"

#include <iostream>

#include <yaml-cpp/yaml.h>

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

} // namespace stools
} // namespace input
