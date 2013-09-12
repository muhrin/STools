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

#include <spl/yaml_schema/YamlSchema.h>
#include <spl/factory/SsLibYamlSchema.h>

// From SPipe
#include <factory/YamlSchema.h>
#include <factory/PipeEngineSchema.h>

// DEFINES //////////////////////////////////////////////

namespace stools {
namespace factory {

///////////////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////////////
typedef ::spl::factory::HeteroMap HeteroMap;

///////////////////////////////////////////////////////////
// CUSTOM MAPS
///////////////////////////////////////////////////////////

struct PipeSettings : HeteroMap
{
  PipeSettings()
  {
    namespace spf = ::spipe::factory;

    addEntry("engine", spf::ENGINE, new spf::Engine())->element()->required();

    ::spl::utility::HeterogeneousMap defaults;
    defaults[spf::ENGINE][spf::SERIAL];
    defaultValue(defaults);
  }
};

struct Build : PipeSettings
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  Build()
  {
    namespace spf = ::spipe::factory;

    ::spl::utility::HeterogeneousMap buildStructuresDefault;
    buildStructuresDefault[spf::NUM] = 1;

    addScalarEntry("rngSeed", spf::RNG_SEED)->element()->defaultValue("time");
    addEntry("buildStructures", spf::BUILD_STRUCTURES,
        (new spf::blocks::BuildStructures())->defaultValue(
            buildStructuresDefault))->required();
    addEntry("writeStructures", spf::WRITE_STRUCTURES,
        new spf::blocks::WriteStructures());

    // Defaults
    BindingType defaultOptions;
    if(getDefault())
      defaultOptions = *getDefault();
    defaultOptions[spf::WRITE_STRUCTURES];
    defaultValue(defaultOptions);
  }
};

struct Search : PipeSettings
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  Search()
  {
    namespace spf = ::spipe::factory;
    namespace ssf = ::spl::factory;

    // Global options
    addScalarEntry("rngSeed", spf::RNG_SEED)->element()->defaultValue("time");
    addScalarEntry("castepExe", ssf::CASTEP_EXE)->element()->defaultValue(
        "castep");

    addEntry("buildStructures", spf::BUILD_STRUCTURES,
        new spf::blocks::BuildStructures());
    addEntry("preGeomOptimise", spf::PRE_GEOM_OPTIMISE,
        new spf::blocks::GeomOptimise());
    addEntry("geomOptimise", spf::GEOM_OPTIMISE,
        new spf::blocks::GeomOptimise());
    addEntry("removeDuplicates", spf::REMOVE_DUPLICATES,
        new spf::blocks::RemoveDuplicates());
    addEntry("keepTopN", spf::KEEP_TOP_N, new spf::blocks::KeepTopN());
    addEntry("keepWithinXPercent", spf::KEEP_WITHIN_X_PERCENT,
        new spf::blocks::KeepWithinXPercent());
    addEntry("writeStructures", spf::WRITE_STRUCTURES,
        new spf::blocks::WriteStructures());
    addEntry("sweepPotentialParams", spf::SWEEP_POTENTIAL_PARAMS,
        new spf::blocks::SweepPotentialParams());
    addEntry("runPotentialParamsQueue", spf::RUN_POTENTIAL_PARAMS_QUEUE,
        new spf::blocks::RunPotentialParamsQueue());
  }
};

}
}

#endif /* STOOLS__FACTORY__YAML_SCHEMA_H */

