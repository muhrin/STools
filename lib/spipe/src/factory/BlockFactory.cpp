/*
 * BlockFactory.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "factory/BlockFactory.h"

#include <spl/potential/Types.h>
#include <spl/utility/UtilityFwd.h>

// Local includes
#include "blocks/BuildStructures.h"
#include "blocks/Clone.h"
#include "blocks/CutAndPaste.h"
#include "blocks/FindSymmetryGroup.h"
#include "blocks/KeepStableCompositions.h"
#include "blocks/KeepTopN.h"
#include "blocks/KeepWithinXPercent.h"
#include "blocks/LoadStructures.h"
#include "blocks/NiggliReduce.h"
#include "blocks/GeomOptimise.h"
#include "blocks/ParamGeomOptimise.h"
#include "blocks/RemoveDuplicates.h"
#include "blocks/RunPotentialParamsQueue.h"
#include "blocks/SearchStoichiometries.h"
#include "blocks/SweepPotentialParams.h"
#include "blocks/WriteStructures.h"
#include "common/CommonData.h"
#include "common/StructureData.h"
#include "common/SharedData.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace factory {

// Alias for accessing keywords namespace
namespace ssf = ::spl::factory;
namespace spb = ::spipe::blocks;
namespace ssbc = ::spl::build_cell;
namespace ssio = ::spl::io;
namespace ssp = ::spl::potential;
namespace ssu = ::spl::utility;

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::BuildStructures & options) const
{
  // Try to construct a structure generator
  ssbc::IStructureGeneratorPtr generator(
      mySplFactory.createStructureBuilder(options));
  if(!generator.get())
    return false;

  blockOut->reset(new spipe::blocks::BuildStructures(options.num, generator));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::Clone & options) const
{
  if(options.giveUniqueNames)
    blockOut->reset(
        new spipe::blocks::Clone(options.times, *options.giveUniqueNames));
  else
    blockOut->reset(new spipe::blocks::Clone(options.times));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::CutAndPaste & options) const
{
  ssf::GenShapeFactory::GenShapePtr genShape;
  if(!mySplFactory.getShapeFactory().createShape(genShape, options.genShape))
    return false;

  ::spipe::blocks::CutAndPaste::Settings settings;
  settings.paste = options.paste;
  settings.separate = options.separate;
  settings.fixUntouched = options.fixUntouched;

  blockOut->reset(new ::spipe::blocks::CutAndPaste(genShape, settings));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::FindSymmetryGroup & options) const
{
  blockOut->reset(new ::spipe::blocks::FindSymmetryGroup());
  return true;
}

#ifdef SPL_WITH_CGAL
bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::KeepStableCompositions & options) const
{
  blockOut->reset(
      new ::spipe::blocks::KeepStableCompositions(options.writeHull));
  return true;
}
#endif

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::KeepTopN & options) const
{
  blockOut->reset(new ::spipe::blocks::KeepTopN(options.num));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::KeepWithinXPercent & options) const
{
  blockOut->reset(new ::spipe::blocks::KeepWithinXPercent(options.percent));
  return false;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const ::std::string & toLoad) const
{
  blockOut->reset(new ::spipe::blocks::LoadStructures(toLoad));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::NiggliReduce & options) const
{
  blockOut->reset(new ::spipe::blocks::NiggliReduce());
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::GeomOptimise & options) const
{
  ssp::IGeomOptimiserPtr optimiser = mySplFactory.createGeometryOptimiser(
      options.optimiser);
  if(!optimiser.get())
    return false;

  const bool potentialIsParameterisable = optimiser->getPotential()
      && optimiser->getPotential()->getParameterisable();

  const ssp::OptimisationSettings settings =
      mySplFactory.createOptimisationSettings(options);

  if(potentialIsParameterisable)
    blockOut->reset(
        new ::spipe::blocks::ParamGeomOptimise(optimiser, settings,
            options.writeSummary));
  else
    blockOut->reset(
        new ::spipe::blocks::GeomOptimise(optimiser, settings,
            options.writeSummary));

  return true;
}

bool
BlockFactory::createBlock(BlockHandle * blockOut,
    const blocks::RemoveDuplicates & options) const
{
  ssu::IStructureComparatorPtr comparator(
      mySplFactory.createStructureComparator(options.comparator));
  if(!comparator.get())
    return false;

  blockOut->reset(new ::spipe::blocks::RemoveDuplicates(comparator));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::RunPotentialParamsQueue & options, BlockHandle subpipe) const
{
  if(!subpipe)
    return false;

  blockOut->reset(
      new ::spipe::blocks::RunPotentialParamsQueue(&options.paramsQueueFile,
          &options.paramsDoneFile, subpipe));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::SearchStoichiometries & options, BlockHandle subpipe) const
{
  if(!subpipe)
    return false;

  ::spipe::blocks::SearchStoichiometries::Options searchOptions;
  searchOptions.atomRanges = options.ranges;
  searchOptions.useSeparateDirectories = options.useSeparateDirs;

  blockOut->reset(
      new ::spipe::blocks::SearchStoichiometries(searchOptions, subpipe));

  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::SweepPotentialParams & options, BlockHandle sweepPipe) const
{
  if(!sweepPipe)
    return false;

  common::ParamRange paramRange;
  if(!paramRange.fromStrings(options.range))
    return false;

  blockOut->reset(
      new ::spipe::blocks::SweepPotentialParams(paramRange, sweepPipe));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * blockOut,
    const blocks::WriteStructures & options) const
{
  ::spl::UniquePtr< ::spipe::blocks::WriteStructures>::Type writeStructures(
      new ::spipe::blocks::WriteStructures());

  writeStructures->setFileType(options.format);
  writeStructures->setWriteMulti(options.multiWrite);

  // Transfer ownership
  *blockOut = writeStructures;
  return true;
}

} // namespace common
} // namespace factory
