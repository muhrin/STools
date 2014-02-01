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
#include "blocks/SeparateAtoms.h"
#include "blocks/SweepPotentialParams.h"
#include "blocks/WriteStructures.h"
#include "common/CommonData.h"
#include "common/StructureData.h"
#include "common/SharedData.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace factory {

// Alias for accessing keywords namespace
namespace ssf = spl::factory;
namespace spb = spipe::blocks;
namespace ssbc = spl::build_cell;
namespace ssio = spl::io;
namespace ssp = spl::potential;
namespace ssu = spl::utility;

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::BuildStructures & options) const
{
  // Try to construct a structure generator
  ssbc::IStructureGeneratorPtr generator(
      mySplFactory.createStructureGenerator(options.generator));
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

  spipe::blocks::CutAndPaste::Settings settings;
  settings.paste = options.paste;
  settings.separate = options.separate;
  settings.fixUntouched = options.fixUntouched;

  spl::UniquePtr< spipe::blocks::CutAndPaste>::Type cutAndPaste(
      new spipe::blocks::CutAndPaste(genShape, settings));
  if(options.pairDistances)
    cutAndPaste->setPairDistances(*options.pairDistances);

  blockOut->reset(cutAndPaste.release());
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::FindSymmetryGroup & options) const
{
  blockOut->reset(new spipe::blocks::FindSymmetryGroup());
  return true;
}

#ifdef SPL_WITH_CGAL
bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::KeepStableCompositions & options) const
{
  blockOut->reset(new spipe::blocks::KeepStableCompositions(options.writeHull));
  return true;
}
#endif

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::KeepTopN & options) const
{
  blockOut->reset(new spipe::blocks::KeepTopN(options.num));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::KeepWithinXPercent & options) const
{
  blockOut->reset(new spipe::blocks::KeepWithinXPercent(options.percent));
  return false;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const std::string & toLoad) const
{
  blockOut->reset(new spipe::blocks::LoadStructures(toLoad));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    const blocks::NiggliReduce & options) const
{
  blockOut->reset(new spipe::blocks::NiggliReduce());
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

  const ssp::OptimisationSettings optParams =
      mySplFactory.createOptimisationSettings(options);

  spipe::blocks::GeomOptimise::Settings settings;
  settings.failAction = options.failAction;
  settings.writeSummary = options.writeSummary;

  if(potentialIsParameterisable)
    blockOut->reset(
        new spipe::blocks::ParamGeomOptimise(optimiser, optParams, settings));
  else
    blockOut->reset(
        new spipe::blocks::GeomOptimise(optimiser, optParams, settings));

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

  blockOut->reset(new spipe::blocks::RemoveDuplicates(comparator));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    blocks::RunPotentialParamsQueue & options) const
{
  if(!options.pipe)
    return false;

  blockOut->reset(
      new spipe::blocks::RunPotentialParamsQueue(&options.paramsQueueFile,
          &options.paramsDoneFile, options.pipe));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    blocks::SearchStoichiometries & options) const
{
  if(!options.pipe)
    return false;

  spipe::blocks::SearchStoichiometries::Options searchOptions;
  searchOptions.atomRanges = options.ranges;
  searchOptions.useSeparateDirectories = options.useSeparateDirs;

  blockOut->reset(
      new spipe::blocks::SearchStoichiometries(searchOptions, options.pipe));

  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    blocks::SeparateAtoms & options) const
{
  spl::UniquePtr< spipe::blocks::SeparateAtoms>::Type sep(
      new spipe::blocks::SeparateAtoms());
  if(options.pairDistances)
    sep->setPairDistances(*options.pairDistances);
  blockOut->reset(sep.release());
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * const blockOut,
    blocks::SweepPotentialParams & options) const
{
  if(!options.pipe)
    return false;

  common::ParamRange paramRange;
  if(!paramRange.fromStrings(options.range))
    return false;

  blockOut->reset(
      new spipe::blocks::SweepPotentialParams(paramRange, options.pipe));
  return true;
}

bool
BlockFactory::createBlock(BlockHandle * blockOut,
    const blocks::WriteStructures & options) const
{
  spl::UniquePtr< spipe::blocks::WriteStructures>::Type writeStructures(
      new spipe::blocks::WriteStructures());

  writeStructures->setFileType(options.format);
  writeStructures->setWriteMulti(options.multiWrite);

  // Transfer ownership
  *blockOut = writeStructures;
  return true;
}

} // namespace common
} // namespace factory
