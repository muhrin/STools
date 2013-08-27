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

#include <spl/build_cell/BuildCellFwd.h>
#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/factory/FactoryError.h>
#include <spl/factory/SsLibFactoryYaml.h>
#include <spl/factory/SsLibYamlKeywords.h>
#include <spl/factory/SsLibYamlSchema.h>
#include <spl/utility/HeterogeneousMap.h>
#include <yaml_schema/SchemaMap.h>

namespace ssbc = ::spl::build_cell;
namespace ssc  = ::spl::common;
namespace ssf = ::spl::factory;
namespace ssu = ::spl::utility;
namespace ssys = ::spl::yaml_schema;

namespace kw = ::spl::factory::sslib_yaml_keywords;

BOOST_AUTO_TEST_CASE(StructureGeneratorTest)
{
  //// Settings ////////////////
  const char simpleStructure[] = "RandomStructure.sslib";
  const ::std::string randStrNode = "randomStructure";

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
