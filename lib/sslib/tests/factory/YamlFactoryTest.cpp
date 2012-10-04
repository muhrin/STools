/*
 * YamlFactoryTest.cpp
 *
 *  Created on: Oct 3, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include <iostream>

#include <boost/test/unit_test.hpp>

#include <yaml-cpp/yaml.h>

#include <build_cell/Types.h>
#include <factory/SsLibFactoryYaml.h>
#include <factory/SsLibYamlKeywords.h>

namespace ssbc = ::sstbx::build_cell;
namespace ssf = ::sstbx::factory;

namespace kw = ::sstbx::factory::sslib_yaml_keywords;

BOOST_AUTO_TEST_CASE(StructureDescriptionTest)
{
  // Settings ////////////////
  const char simpleStructure[] = "SimpleStructure.sslib";

  ssf::SsLibFactoryYaml factory;

  YAML::Node loadedNode = YAML::LoadFile(simpleStructure);

  YAML::Node * structureNode = &loadedNode;
  YAML::Node strNode;
  if(loadedNode[kw::STRUCTURE])
  {
    strNode = loadedNode[kw::STRUCTURE];
    structureNode = &strNode;
  }

  ssbc::StructureDescriptionPtr strGen = factory.createStructureDescription(*structureNode);  
}
