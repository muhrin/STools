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
#include <spl/yaml_schema/YamlSchema.h>

#include "blocks/RunPotentialParamsQueue.h"
#include "blocks/WriteStructures.h"
#include "factory/MapEntries.h"

// DEFINES //////////////////////////////////////////////

namespace spipe {
namespace factory {

///////////////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////////////
typedef ::spl::factory::HeteroMap HeteroMap;

///////////////////////////////////////////////////////////
// CUSTOM MAPS
///////////////////////////////////////////////////////////
namespace blocks {

struct BuildStructures : ::spl::factory::builder::Builder
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  BuildStructures()
  {
    addScalarEntry("num", NUM)->element()->defaultValue(1);
  }
};

struct Clone : public HeteroMap
{
  Clone()
  {
  }
};

struct CutAndPaste : HeteroMap
{
  CutAndPaste()
  {
    addEntry("shape", ::spl::factory::GEN_SHAPE,
        new ::spl::factory::builder::GenShape())->element()->required();
    addScalarEntry("paste", CUT_AND_PASTE__PASTE)->element()->defaultValue(true);
    addScalarEntry("separate", CUT_AND_PASTE__SEPARATE)->element()->defaultValue(
        true);
    addScalarEntry("fixUntouched", CUT_AND_PASTE__FIX_UNTOUCHED)->element()->defaultValue(
        true);
  }
};

struct FindSymmetryGroup : HeteroMap
{
};

struct KeepStableCompositions : HeteroMap
{
  KeepStableCompositions()
  {
    addScalarEntry("writeHull", WRITE_HULL)->element()->defaultValue(false);
  }
};

struct KeepTopN : HeteroMap
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  KeepTopN()
  {
    addScalarEntry("num", NUM)->element()->defaultValue(1);
  }
};

struct KeepWithinXPercent : HeteroMap
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  KeepWithinXPercent()
  {
    addScalarEntry("percent", PERCENT)->element()->defaultValue(5.0);
  }
};

struct LoadStructures : spl::yaml_schema::SchemaScalar< ::std::string>
{
};

struct NiggliReduce : HeteroMap
{
  NiggliReduce()
  {
  }
};

struct GeomOptimise : ::spl::factory::OptimisationSettings
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  GeomOptimise()
  {
    addEntry("optimiser", ::spl::factory::OPTIMISER,
        new ::spl::factory::Optimiser());
    addScalarEntry("writeSummary", ::spl::factory::WRITE_SUMMARY)->element()->defaultValue(
        false);
  }
};

struct RemoveDuplicates : HeteroMap
{
  typedef ::spl::utility::HeterogeneousMap BindingType;
  RemoveDuplicates()
  {
    addEntry("comparator", ::spl::factory::COMPARATOR,
        new ::spl::factory::Comparator());

    //::spl::utility::HeterogeneousMap defaultOptions;
    //defaultOptions[::spl::factory::COMPARATOR];
    //defaultValue(defaultOptions);
  }
};

struct RunPotentialParamsQueue : HeteroMap
{
  RunPotentialParamsQueue()
  {
    addScalarEntry("queueFile", QUEUE_FILE)->element()->defaultValue(
        spipe::blocks::RunPotentialParamsQueue::DEFAULT_PARAMS_QUEUE_FILE);
    addScalarEntry("doneFile", DONE_FILE)->element()->defaultValue(
        spipe::blocks::RunPotentialParamsQueue::DEFAULT_PARAMS_DONE_FILE);
  }
};

struct SearchStoichiometries : HeteroMap
{
  typedef ::spl::yaml_schema::SchemaScalar<
      ::spl::build_cell::AtomsDescription::CountRange> CountRange;
  typedef ::spl::yaml_schema::SchemaHomoMap< CountRange> AtomRanges;

  SearchStoichiometries()
  {
    addEntry("ranges", ATOM_RANGES, new AtomRanges());
    addScalarEntry("useSeparateDirs", USE_SEPARATE_DIRS)->element()->defaultValue(
        false);
  }
};

struct SweepPotentialParams : HeteroMap
{
  SweepPotentialParams()
  {
    addEntry("range", PARAM_RANGE,
        new ::spl::yaml_schema::SchemaWrapper<
            ::spl::yaml::VectorAsString< ::std::string> >)->required();
  }
};

struct WriteStructures : HeteroMap
{
  WriteStructures()
  {
    addScalarEntry("multiWrite", MULTI_WRITE)->element()->defaultValue(
        ::spipe::blocks::WriteStructures::WRITE_MULTI_DEFAULT);
    addScalarEntry("format", FORMAT)->element()->defaultValue(
        ::spipe::blocks::WriteStructures::FORMAT_DEFAULT);
  }
};

} // namespace blocks
}
}

#endif /* SPIPE__FACTORY__YAML_SCHEMA_H */

