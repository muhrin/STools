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


#include <spl/factory/SsLibYamlSchema.h>
#include <spl/utility/HeterogeneousMap.h>
#include <spl/yaml_schema/YamlSchema.h>

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

struct GeomOptimise : ::spl::yaml_schema::SchemaHeteroMap
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  GeomOptimise()
  {
    addEntry(
      "optimiser",
      ::spl::factory::OPTIMISER,
      new ::spl::factory::Optimiser()
    );
    addEntry(
      "potential",
      ::spl::factory::POTENTIAL,
      new ::spl::factory::Potential()
    );
    addScalarEntry("pressure", ::spl::factory::PRESSURE);
  }
};

struct LowestEnergy : public ::spl::yaml_schema::SchemaHeteroMap
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  LowestEnergy()
  {
    addScalarEntry("keepWithin", KEEP_WITHIN)->element()->defaultValue(0.1);
    addScalarEntry("keepTop", KEEP_TOP);
  }
};

struct ParamSweep : public ::spl::yaml_schema::SchemaHeteroMap
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  ParamSweep()
  {
    addEntry(
      "range",
      PARAM_RANGE,
      new ::spl::yaml_schema::SchemaWrapper< ::spl::yaml::VectorAsString< ::std::string> >
    )->required();
  }
};

struct RandomStructure : public ::spl::factory::builder::Builder
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  RandomStructure()
  {
    addScalarEntry("num", NUM)->element()->defaultValue(100);
  }
};

struct RemoveDuplicates : public ::spl::yaml_schema::SchemaHeteroMap
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  RemoveDuplicates()
  {
    addEntry(
      "comparator",
      ::spl::factory::COMPARATOR,
      new ::spl::factory::Comparator()
    );

    //::spl::utility::HeterogeneousMap defaultOptions;
    //defaultOptions[::spl::factory::COMPARATOR];
    //defaultValue(defaultOptions);
  }
};

struct WriteStructure : ::spl::yaml_schema::SchemaHeteroMap
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  WriteStructure()
  {
    addScalarEntry("multiWrite", MULTI_WRITE)->
      element()->defaultValue(::spipe::blocks::WriteStructure::WRITE_MULTI_DEFAULT);
    addScalarEntry("fileType", FILE_TYPE);
  }
};

} // namespace blocks

namespace pipes {

struct Search : ::spl::yaml_schema::SchemaHeteroMap
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  Search()
  {
    // Simple options
    addScalarEntry("pressure", ::spl::factory::PRESSURE);
    addEntry(
      "potential",
      ::spl::factory::POTENTIAL,
      new ::spl::factory::Potential()
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

