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

#include <yaml-cpp/yaml.h>

#include <build_cell/BuildCellFwd.h>
#include <common/AtomSpeciesDatabase.h>
#include <factory/FactoryError.h>
#include <factory/SsLibFactoryYaml.h>
#include <factory/SsLibYamlKeywords.h>
#include <factory/SsLibYamlSchema.h>
#include <utility/HeterogeneousMap.h>
#include <yaml_schema/SchemaMap.h>

namespace ssbc = ::sstbx::build_cell;
namespace ssc  = ::sstbx::common;
namespace ssf = ::sstbx::factory;
namespace ssu = ::sstbx::utility;
namespace ssys = ::sstbx::yaml_schema;

namespace kw = ::sstbx::factory::sslib_yaml_keywords;

BOOST_AUTO_TEST_CASE(StructureGeneratorTest)
{
  //// Settings ////////////////
  const char simpleStructure[] = "RandomStructure.sslib";
  const ::std::string randStrNode("randomStructure");

  ssc::AtomSpeciesDatabase speciesDb;

  ssf::SsLibFactoryYaml factory(speciesDb);

  const YAML::Node loadedNode = YAML::LoadFile(simpleStructure);

  ssys::SchemaParse parse;
  ssys::SchemaHeteroMap schema;
  schema.addEntry("builder", ssf::BUILDER, new ssf::builder::Builder());
  ssu::HeterogeneousMap builderMap;
  schema.nodeToValue(parse, builderMap, loadedNode, true);
  parse.printErrors();

  try
  {
    ssbc::IStructureGeneratorPtr strGen = factory.createStructureGenerator(builderMap);
  }
  catch(const ssf::FactoryError & e)
  {
    ::std::cout << ::boost::diagnostic_information(e) << ::std::endl;
  }
}
