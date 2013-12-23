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
#include <spl/factory/SsLibElements.h>

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
#include "factory/MapEntries.h"

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

struct BlockMap
{
  typedef bool
  (BlockFactory::*CreationFunction)(BlockHandle * const blockOut,
      const BlockFactory::OptionsMap & options) const;
  typedef ::std::map< BlockFactory::Blocks::Value, CreationFunction> CreationMap;

  BlockMap()
  {
    map[BlockFactory::Blocks::BUILD_STRUCTURES] =
        &BlockFactory::createBuildStructuresBlock;
    map[BlockFactory::Blocks::CLONE_BLOCK] = &BlockFactory::createCloneBlock;
    map[BlockFactory::Blocks::CUT_AND_PASTE] =
        &BlockFactory::createCutAndPasteBlock;
    map[BlockFactory::Blocks::FIND_SYMMETRY] =
        &BlockFactory::createFindSymmetryGroupBlock;
#ifdef SSLIB_USE_CGAL
    map[BlockFactory::Blocks::KEEP_STABLE_COMPOSITIONS] =
        &BlockFactory::createKeepStableCompositionsBlock;
#endif
    map[BlockFactory::Blocks::KEEP_TOP_N] =
        &BlockFactory::createKeepTopNBlock;
    map[BlockFactory::Blocks::KEEP_WITHIN_X_PERCENT] =
        &BlockFactory::createKeepWithinXPercentBlock;
    //map[BlockFactory::Blocks::LOAD_STRUCTURES] =
    //    &BlockFactory::createLoadStructuresBlock;
    map[BlockFactory::Blocks::NIGGLI_REDUCE] =
        &BlockFactory::createNiggliReduceBlock;
    map[BlockFactory::Blocks::GEOM_OPTIMISE] =
        &BlockFactory::createGeomOptimiseBlock;
    map[BlockFactory::Blocks::REMOVE_DUPLICATES] =
        &BlockFactory::createRemoveDuplicatesBlock;
    map[BlockFactory::Blocks::WRITE_STRUCTURES] =
        &BlockFactory::createWriteStructuresBlock;
  }
  CreationMap map;
};

static BlockMap BLOCK_MAP;

BlockFactory::BlockFactory(::spl::common::AtomSpeciesDatabase & speciesDb) :
    mySplFactory(speciesDb)
{
}

bool
BlockFactory::createBlock(const Blocks::Value block,
    const OptionsMap & options, BlockHandle * const blockOut) const
{
  BlockMap::CreationMap::const_iterator it = BLOCK_MAP.map.find(block);
  if(it == BLOCK_MAP.map.end())
    return false;

  return (this->*it->second)(blockOut, options);
}

bool
BlockFactory::createBuildStructuresBlock(BlockHandle * const blockOut,
    const OptionsMap & options) const
{
  // Try to construct a structure generator
  ssbc::IStructureGeneratorPtr generator(
      mySplFactory.createStructureBuilder(options));
  if(!generator.get())
    return false;

  const int * const numToGenerate = options.find(NUM);
  if(!numToGenerate)
    return false;

  blockOut->reset(new blocks::BuildStructures(*numToGenerate, generator));
  return true;
}

bool
BlockFactory::createCloneBlock(BlockHandle * const blockOut,
    const OptionsMap & options) const
{
  const int * const times = options.find(factory::CLONE__TIMES);
  if(!times)
    return false;

  const bool * const giveUniqueNames = options.find(
      factory::CLONE__GIVE_UNIQUE_NAMES);
  if(giveUniqueNames)
    blockOut->reset(new blocks::Clone(*times, *giveUniqueNames));
  else
    blockOut->reset(new blocks::Clone(*times));
  return true;
}

bool
BlockFactory::createCutAndPasteBlock(BlockHandle * const blockOut,
    const OptionsMap & options) const
{
  const OptionsMap * const shapeOptions = options.find(ssf::GEN_SHAPE);
  if(!shapeOptions)
    return false;

  ssf::GenShapeFactory::GenShapePtr genShape;
  if(!mySplFactory.getShapeFactory().createShape(genShape, *shapeOptions))
    return false;

  blocks::CutAndPaste::Settings settings;
  {
    const bool * const paste = options.find(factory::CUT_AND_PASTE__PASTE);
    if(paste)
      settings.paste = *paste;
  }
  {
    const bool * const separate = options.find(
        factory::CUT_AND_PASTE__SEPARATE);
    if(separate)
      settings.separate = *separate;
  }
  {
    const bool * const fixUntouched = options.find(
        factory::CUT_AND_PASTE__FIX_UNTOUCHED);
    if(fixUntouched)
      settings.fixUntouched = *fixUntouched;
  }

  blockOut->reset(new blocks::CutAndPaste(genShape, settings));
  return true;
}

bool
BlockFactory::createFindSymmetryGroupBlock(BlockHandle * const blockOut,
    const OptionsMap & options) const
{
  blockOut->reset(new blocks::FindSymmetryGroup());
  return true;
}

#ifdef SSLIB_USE_CGAL
bool
BlockFactory::createKeepStableCompositionsBlock(BlockHandle * const blockOut,
    const OptionsMap & options) const
{
  bool writeHull = false;
  {
    const bool * const writeHullOpt = options.find(WRITE_HULL);
    if(writeHullOpt)
      writeHull = *writeHullOpt;
  }

  blockOut->reset(new blocks::KeepStableCompositions(writeHull));
  return true;
}
#endif

bool
BlockFactory::createKeepTopNBlock(BlockHandle * const blockOut,
    const OptionsMap & options) const
{
  const int * const num = options.find(NUM);
  if(num)
  {
    blockOut->reset(new blocks::KeepTopN(*num));
    return true;
  }
  return false;
}

bool
BlockFactory::createKeepWithinXPercentBlock(BlockHandle * const blockOut,
    const OptionsMap & options) const
{
  const double * const percent = options.find(PERCENT);
  if(percent)
  {
    blockOut->reset(new blocks::KeepWithinXPercent(*percent));
    return true;
  }

  return false;
}

bool
BlockFactory::createLoadStructuresBlock(BlockHandle * const blockOut,
    const ::std::string & toLoad) const
{
  blockOut->reset(new blocks::LoadStructures(toLoad));
  return true;
}

bool
BlockFactory::createNiggliReduceBlock(BlockHandle * const blockOut,
    const OptionsMap & options) const
{
  blockOut->reset(new blocks::NiggliReduce());
  return true;
}

bool
BlockFactory::createGeomOptimiseBlock(BlockHandle * const blockOut,
    const OptionsMap & options) const
{
  const OptionsMap * const optimiserOptions = options.find(ssf::OPTIMISER);
  if(!optimiserOptions)
    return false;

  ssp::IGeomOptimiserPtr optimiser = mySplFactory.createGeometryOptimiser(
      *optimiserOptions);
  if(!optimiser.get())
    return false;

  const bool potentialIsParameterisable = optimiser->getPotential()
      && optimiser->getPotential()->getParameterisable();

  const ssp::OptimisationSettings settings =
      mySplFactory.createOptimisationSettings(options);

  bool writeSummary = false;
  {
    const bool * const writeSummaryOpt = options.find(ssf::WRITE_SUMMARY);
    if(writeSummaryOpt)
      writeSummary = *writeSummaryOpt;
  }

  if(potentialIsParameterisable)
    blockOut->reset(
        new blocks::ParamGeomOptimise(optimiser, settings, writeSummary));
  else
    blockOut->reset(
        new blocks::GeomOptimise(optimiser, settings, writeSummary));

  return true;
}

bool
BlockFactory::createRemoveDuplicatesBlock(BlockHandle * blockOut,
    const OptionsMap & options) const
{
  // Check for setting relating to the comparator
  const OptionsMap * const comparatorOptions = options.find(ssf::COMPARATOR);
  if(!comparatorOptions)
    return false;

  ssu::IStructureComparatorPtr comparator(
      mySplFactory.createStructureComparator(*comparatorOptions));
  if(!comparator.get())
    return false;

  blockOut->reset(new blocks::RemoveDuplicates(comparator));
  return true;
}

bool
BlockFactory::createRunPotentialParamsQueueBlock(BlockHandle * const blockOut,
    const OptionsMap & options, BlockHandle subpipe) const
{
  if(!subpipe)
    return false;

  const ::std::string * const queueFile = options.find(QUEUE_FILE);
  const ::std::string * const doneFile = options.find(DONE_FILE);

  blockOut->reset(
      new blocks::RunPotentialParamsQueue(queueFile, doneFile, subpipe));
  return true;
}

bool
BlockFactory::createSearchStoichiometriesBlock(BlockHandle * const blockOut,
    const OptionsMap & options, BlockHandle subpipe) const
{
  typedef ::std::map< ::std::string,
      ::spl::build_cell::AtomsDescription::CountRange> AtomRanges;

  if(!subpipe)
    return false;

  const AtomRanges * const atomRanges = options.find(ATOM_RANGES);
  if(!atomRanges)
    return false;

  blocks::SearchStoichiometries::Options searchOptions;
  searchOptions.atomRanges = *atomRanges;

  {
    const bool * const separateDirectories = options.find(USE_SEPARATE_DIRS);
    if(separateDirectories)
      searchOptions.useSeparateDirectories = *separateDirectories;
  }

  blockOut->reset(new blocks::SearchStoichiometries(searchOptions, subpipe));

  return true;
}

bool
BlockFactory::createSweepPotentialParamsBlock(BlockHandle * const blockOut,
    const OptionsMap & options, BlockHandle sweepPipe) const
{
  const ::std::vector< ::std::string> * const paramStrings = options.find(
      PARAM_RANGE);
  if(!paramStrings)
    return false;

  common::ParamRange paramRange;
  if(!paramRange.fromStrings(*paramStrings))
    return false;

  blockOut->reset(new blocks::SweepPotentialParams(paramRange, sweepPipe));
  return true;
}

bool
BlockFactory::createWriteStructuresBlock(BlockHandle * blockOut,
    const OptionsMap & options) const
{
  ::spl::UniquePtr< blocks::WriteStructures>::Type writeStructures(
      new blocks::WriteStructures());

  const bool * const multiWrite = options.find(MULTI_WRITE);
  if(multiWrite)
    writeStructures->setWriteMulti(*multiWrite);

  const ::std::string * const format = options.find(FORMAT);
  if(format)
    writeStructures->setFileType(*format);

  // Transfer ownership
  *blockOut = writeStructures;
  return true;
}

} // namespace common
} // namespace factory
