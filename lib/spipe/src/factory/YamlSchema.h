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

#include <schemer/Schemer.h>

#include <spl/factory/SsLibYamlSchema.h>

#include "blocks/RunPotentialParamsQueue.h"
#include "blocks/WriteStructures.h"

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

struct BuildStructures : public ::spl::factory::builder::Builder
{
  int num;
};

SCHEMER_MAP(BuildStructuresSchema, BuildStructures)
{
  element("num", &BuildStructures::num)->defaultValue(1);
}
;

struct Clone
{
  int times;
  ::boost::optional< bool> giveUniqueNames;
};

SCHEMER_MAP(CloneSchema, Clone)
{
  element("times", &Clone::times);
  element("giveUniqueNames", &Clone::giveUniqueNames);
}

struct CutAndPaste
{
  ::spl::factory::builder::GenShape genShape;
  bool paste;
  bool separate;
  bool fixUntouched;
};

SCHEMER_MAP(CutAndPasteSchema, CutAndPaste)
{
  element("shape", &CutAndPaste::genShape);
  element("paste", &CutAndPaste::paste)->defaultValue(true);
  element("separate", &CutAndPaste::separate)->defaultValue(true);
  element("fixUntouched", &CutAndPaste::fixUntouched)->defaultValue(true);
}

struct FindSymmetryGroup
{
};

SCHEMER_MAP(FindSymmetryGroupSchema, FindSymmetryGroup)
{
}

struct KeepStableCompositions
{
  bool writeHull;
};

SCHEMER_MAP(KeepStableCompositionsSchema, KeepStableCompositions)
{
  element("writeHull", &KeepStableCompositions::writeHull)->defaultValue(false);
}

struct KeepTopN
{
  int num;
};

SCHEMER_MAP(KeepTopNSchema, KeepTopN)
{
  element("num", &KeepTopN::num)->defaultValue(1);
}

struct KeepWithinXPercent
{
  double percent;
};

SCHEMER_MAP(KeepWithinXPercentSchema, KeepWithinXPercent)
{
  element("percent", &KeepWithinXPercent::percent)->defaultValue(5);
}

typedef ::schemer::Scalar< ::std::string> LoadStructures;

struct NiggliReduce
{
};

SCHEMER_MAP(NiggliReduceSchema, NiggliReduce)
{
}

struct GeomOptimise : ::spl::factory::OptimiserSettings
{
  ::spl::factory::Optimiser optimiser;
  bool writeSummary;
};

SCHEMER_MAP(GeomOptimiseSchema, GeomOptimise)
{
  extends< ::spl::factory::OptimiserSettingsSchema>();
  element("optimiser", &GeomOptimise::optimiser);
  element("writeSummary", &GeomOptimise::writeSummary)->defaultValue(false);
}

struct RemoveDuplicates
{
  ::spl::factory::Comparator comparator;
};

SCHEMER_MAP(RemoveDuplicatesSchema, RemoveDuplicates)
{
  element("comparator", &RemoveDuplicates::comparator);
}

struct RunPotentialParamsQueue
{
  ::std::string paramsQueueFile;
  ::std::string paramsDoneFile;
};

SCHEMER_MAP(RunPotentialParamsQueueSchema, RunPotentialParamsQueue)
{
  element("queueFile", &RunPotentialParamsQueue::paramsQueueFile)->defaultValue(
      spipe::blocks::RunPotentialParamsQueue::DEFAULT_PARAMS_QUEUE_FILE);
  element("doneFile", &RunPotentialParamsQueue::paramsDoneFile)->defaultValue(
      spipe::blocks::RunPotentialParamsQueue::DEFAULT_PARAMS_DONE_FILE);
}

struct SearchStoichiometries
{
  ::std::map< ::std::string, ::spl::build_cell::AtomsDescription::CountRange> ranges;
  bool useSeparateDirs;
};

SCHEMER_MAP(SearchStoichiometriesSchema, SearchStoichiometries)
{
  element("ranges", &SearchStoichiometries::ranges);
  element("useSeparateDirs", &SearchStoichiometries::useSeparateDirs)->defaultValue(
      false);
}

struct SweepPotentialParams
{
  ::std::vector< ::std::string> range;
};

SCHEMER_MAP(SweepPotentialParamsRange, SweepPotentialParams)
{
  element< ::spl::factory::StringsVector>("range",
      &SweepPotentialParams::range);
}

struct WriteStructures
{
  ::std::string format;
  bool multiWrite;
};

SCHEMER_MAP(WriteStructuresSchema, WriteStructures)
{
  element("format", &WriteStructures::format)->defaultValue(
      ::spipe::blocks::WriteStructures::FORMAT_DEFAULT);
  element("multiWrite", &WriteStructures::multiWrite)->defaultValue(
      ::spipe::blocks::WriteStructures::WRITE_MULTI_DEFAULT);
}

} // namespace blocks
}
}

#endif /* SPIPE__FACTORY__YAML_SCHEMA_H */

