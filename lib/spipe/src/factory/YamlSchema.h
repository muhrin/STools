/*
 * YamlSchema.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SPIPE__FACTORY__YAML_SCHEMA_H
#define SPIPE__FACTORY__YAML_SCHEMA_H

// INCLUDES /////////////////////////////////////////////

// From SSLib
#include <factory/SsLibYamlSchema.h>
#include <utility/HeterogeneousMap.h>
#include <yaml_schema/YamlSchema.h>

#include "blocks/WriteStructure.h"
#include "factory/MapEntries.h"

// DEFINES //////////////////////////////////////////////

namespace spipe {
namespace factory {

///////////////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////
// CUSTOM MAPS
///////////////////////////////////////////////////////////
namespace blocks {

struct GeomOptimise : ::sstbx::yaml_schema::SchemaHeteroMap
{
  typedef ::sstbx::utility::HeterogeneousMap BindingType;
  GeomOptimise()
  {
    addEntry(
      "optimiser",
      ::sstbx::factory::OPTIMISER,
      new ::sstbx::factory::Optimiser()
    );
    addEntry(
      "potential",
      ::sstbx::factory::POTENTIAL,
      new ::sstbx::factory::Potential()
    );
    addScalarEntry("pressure", ::sstbx::factory::PRESSURE);
  }
};

struct LowestEnergy : public ::sstbx::yaml_schema::SchemaHeteroMap
{
  typedef ::sstbx::utility::HeterogeneousMap BindingType;
  LowestEnergy()
  {
    addScalarEntry("keepWithin", KEEP_WITHIN)->element()->defaultValue(0.1);
    addScalarEntry("keepTop", KEEP_TOP);
  }
};

struct ParamSweep : public ::sstbx::yaml_schema::SchemaHeteroMap
{
  typedef ::sstbx::utility::HeterogeneousMap BindingType;
  ParamSweep()
  {
    addEntry(
      "range",
      PARAM_RANGE,
      new ::sstbx::yaml_schema::SchemaWrapper< ::sstbx::yaml::VectorAsString< ::std::string> >
    )->required();
  }
};

struct RandomStructure : public ::sstbx::factory::builder::Builder
{
  typedef ::sstbx::utility::HeterogeneousMap BindingType;
  RandomStructure()
  {
    addScalarEntry("num", NUM)->element()->defaultValue(100);
  }
};

struct RemoveDuplicates : public ::sstbx::yaml_schema::SchemaHeteroMap
{
  typedef ::sstbx::utility::HeterogeneousMap BindingType;
  RemoveDuplicates()
  {
    addEntry(
      "comparator",
      ::sstbx::factory::COMPARATOR,
      new ::sstbx::factory::Comparator()
    );

    //::sstbx::utility::HeterogeneousMap defaultOptions;
    //defaultOptions[::sstbx::factory::COMPARATOR];
    //defaultValue(defaultOptions);
  }
};

struct WriteStructure : ::sstbx::yaml_schema::SchemaHeteroMap
{
  typedef ::sstbx::utility::HeterogeneousMap BindingType;
  WriteStructure()
  {
    addScalarEntry("multiWrite", MULTI_WRITE)->
      element()->defaultValue(::spipe::blocks::WriteStructure::WRITE_MULTI_DEFAULT);
    addScalarEntry("fileType", FILE_TYPE);
  }
};

} // namespace blocks

namespace pipes {

struct Search : ::sstbx::yaml_schema::SchemaHeteroMap
{
  typedef ::sstbx::utility::HeterogeneousMap BindingType;
  Search()
  {
    // Simple options
    addScalarEntry("pressure", ::sstbx::factory::PRESSURE);
    addEntry(
      "potential",
      ::sstbx::factory::POTENTIAL,
      new ::sstbx::factory::Potential()
    );

    // Fine-tune options
    addEntry("randomStructure", RANDOM_STRUCTURE, new blocks::RandomStructure());
    addEntry("preGeomOptimise", PRE_GEOM_OPTIMISE, new blocks::GeomOptimise());
    addEntry("geomOptimise", GEOM_OPTIMISE, new blocks::GeomOptimise());
    addEntry("removeDuplicates", REMOVE_DUPLICATES, new blocks::RemoveDuplicates());
    addEntry("lowestEnergy", LOWEST_ENERGY, new blocks::LowestEnergy());
    addEntry("output", WRITE_STRUCTURES, new blocks::WriteStructure());

    // Defaults

  }
};

} // namespace pipes

}
}

#endif /* SPIPE__FACTORY__YAML_SCHEMA_H */

