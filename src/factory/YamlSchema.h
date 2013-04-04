/*
 * YamlSchema.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef STOOLS__FACTORY__YAML_SCHEMA_H
#define STOOLS__FACTORY__YAML_SCHEMA_H

// INCLUDES /////////////////////////////////////////////

// From SsLib
#include <yaml_schema/YamlSchema.h>
#include <factory/SsLibYamlSchema.h>

// From SPipe
#include <factory/YamlSchema.h>

// DEFINES //////////////////////////////////////////////

namespace stools {
namespace factory {

///////////////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////
// CUSTOM MAPS
///////////////////////////////////////////////////////////

struct Build : public ::sstbx::yaml_schema::SchemaHeteroMap
{
  typedef ::sstbx::utility::HeterogeneousMap BindingType;
  Build()
  {
    namespace spf = ::spipe::factory;

    ::sstbx::utility::HeterogeneousMap randStructureDefault;
    randStructureDefault[spf::NUM] = 1;

    addEntry(
      "randomStructures",
      spf::RANDOM_STRUCTURE,
      (new spf::blocks::RandomStructure())->defaultValue(randStructureDefault)
    )->required();
    addEntry("output", spf::WRITE_STRUCTURES, new spf::blocks::WriteStructure());

    // Defaults
    BindingType defaultOptions;
    defaultOptions[spf::WRITE_STRUCTURES];
    defaultValue(defaultOptions);
  }
};

struct Search : public ::sstbx::yaml_schema::SchemaHeteroMap
{
  typedef ::sstbx::utility::HeterogeneousMap BindingType;
  Search()
  {
    namespace spf = ::spipe::factory;

    addEntry(
      "paramSweep",
      spf::PARAM_SWEEP,
      new spf::blocks::ParamSweep()
    );
    addEntry(
      "randomStructures",
      spf::RANDOM_STRUCTURE,
      new spf::blocks::RandomStructure()
    );
    addEntry(
      "geomOptimise",
      spf::GEOM_OPTIMISE,
      new spf::blocks::GeomOptimise()
    );
    addEntry(
      "preGeomOptimise",
      spf::PRE_GEOM_OPTIMISE,
      new spf::blocks::GeomOptimise()
    );
    addEntry(
      "lowestEnergy",
      spf::LOWEST_ENERGY,
      new spf::blocks::LowestEnergy()
    );
    addEntry(
      "removeDuplicates",
      spf::REMOVE_DUPLICATES,
      new spf::blocks::RemoveDuplicates()
    );
    addEntry(
      "output",
      spf::WRITE_STRUCTURES,
      new spf::blocks::WriteStructure()
    );

    // Defaults

  }
};

}
}

#endif /* STOOLS__FACTORY__YAML_SCHEMA_H */

