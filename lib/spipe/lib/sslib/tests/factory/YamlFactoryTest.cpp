/*
 * YamlFactoryTest.cpp
 *
 *  Created on: Oct 3, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "sslibtest.h"

#include <iostream>

#include <boost/exception/diagnostic_information.hpp>

#include <spl/build_cell/BuildCellFwd.h>
#include <spl/factory/FactoryError.h>
#include <spl/factory/SsLibFactoryYaml.h>
#include <spl/factory/SsLibYamlSchema.h>

using namespace spl;

BOOST_AUTO_TEST_CASE(StructureGeneratorTest)
{
  //// Settings ////////////////
  const char simpleStructure[] = "RandomStructure.sslib";

  factory::Factory factory;

  const YAML::Node loadedNode = YAML::LoadFile(simpleStructure);

  factory::builder::StructureGeneratorSchema schema;
  factory::builder::StructureGenerator generator;
  schemer::ParseLog log;
  schema.nodeToValue(loadedNode, &generator, &log);
  log.printErrors();

  try
  {
    build_cell::IStructureGeneratorPtr strGen = factory.createStructureGenerator(generator);
  }
  catch(const factory::FactoryError & e)
  {
    std::cout << boost::diagnostic_information(e) << std::endl;
  }
}
