/*
 * Factory.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "factory/StFactory.h"


#include <spl/factory/SsLibElements.h>
#include <spl/math/Random.h>
#include <spl/potential/OptimisationSettings.h>
#include <spl/utility/UtilityFwd.h>

// From SPipe
#include <blocks/KeepTopN.h>

// Local includes
#include "factory/MapEntries.h"


// NAMESPACES ////////////////////////////////

namespace stools {
namespace factory {

// Alias for accessing keywords namespace
namespace sp = ::spipe;
namespace spb = sp::blocks;
namespace spf = sp::factory;

namespace ssm = ::spl::math;
namespace ssf = ::spl::factory;
namespace ssp = ::spl::potential;

const Factory::BlockHandle Factory::NULL_HANDLE;

Factory::BlockHandle
Factory::createBuildPipe(const OptionsMap & options) const
{
  const OptionsMap * const buildStructuresOptions = options.find(spf::BUILD_STRUCTURES);
  if(!buildStructuresOptions)
    return NULL_HANDLE;

  BlockHandle startBlock;
  if(!myBlockFactory.createBuildStructuresBlock(&startBlock, *buildStructuresOptions))
    return NULL_HANDLE;

  // Keep track of the last block so we can connect everything up
  BlockHandle block, lastBlock = startBlock;

  const OptionsMap * const outputOptions = options.find(spf::WRITE_STRUCTURES);
  if(outputOptions)
  {
    if(myBlockFactory.createWriteStructuresBlock(&block, *outputOptions))
      lastBlock = lastBlock->connect(block);
    else
      return NULL_HANDLE;
  }

  return startBlock;
}

Factory::BlockHandle
Factory::createSearchPipe(const OptionsMap & options) const
{
  const OptionsMap * const buildStructuresOptions = options.find(spf::BUILD_STRUCTURES);
  if(!buildStructuresOptions)
    return NULL_HANDLE;

  BlockHandle startBlock;
  if(!myBlockFactory.createBuildStructuresBlock(&startBlock, *buildStructuresOptions))
    return NULL_HANDLE;

  // Keep track of the last block so we can connect everything up
  BlockHandle block, lastBlock = startBlock;

  const OptionsMap * const preGeomOptimiseOptions = options.find(spf::PRE_GEOM_OPTIMISE);
  if(preGeomOptimiseOptions)
  {
    if(myBlockFactory.createGeomOptimiseBlock(&block, *preGeomOptimiseOptions))
      lastBlock = lastBlock->connect(block);
    else
      return NULL_HANDLE;
  }

  const OptionsMap * const geomOptimiseOptions = options.find(spf::GEOM_OPTIMISE);
  if(geomOptimiseOptions)
  {
    if(myBlockFactory.createGeomOptimiseBlock(&block, *geomOptimiseOptions))
      lastBlock = lastBlock->connect(block);
    else
      return NULL_HANDLE;
  }

  const OptionsMap * const removeDuplicatesOptions = options.find(spf::REMOVE_DUPLICATES);
  if(removeDuplicatesOptions)
  {
    if(myBlockFactory.createRemoveDuplicatesBlock(&block, *removeDuplicatesOptions))
      lastBlock = lastBlock->connect(block);
    else
      return NULL_HANDLE;
  }

  const OptionsMap * const keepWithinXPercentOptions = options.find(spf::KEEP_WITHIN_X_PERCENT);
  if(keepWithinXPercentOptions)
  {
    if(myBlockFactory.createKeepWithinXPercentBlock(&block, *keepWithinXPercentOptions))
      lastBlock = lastBlock->connect(block);
    else
      return NULL_HANDLE;
  }

  const OptionsMap * const keepTopNOptions = options.find(spf::KEEP_TOP_N);
  if(keepTopNOptions)
  {
    if(myBlockFactory.createKeepTopNBlock(&block, *keepTopNOptions))
      lastBlock = lastBlock->connect(block);
    else
      return NULL_HANDLE;
  }

  // Find out what the symmetry group is
  if(myBlockFactory.createFindSymmetryGroupBlock(&block, OptionsMap()))
    lastBlock = lastBlock->connect(block);
  else
    return NULL_HANDLE;

  const OptionsMap * const writeStructuresOptions = options.find(spf::WRITE_STRUCTURES);
  if(writeStructuresOptions)
  {
    if(myBlockFactory.createWriteStructuresBlock(&block, *writeStructuresOptions))
    lastBlock = lastBlock->connect(block);
  }

  // Finally tack on a lowest energy block to make sure that only one structure
  // comes out the end in all eventualities
  lastBlock = lastBlock->connect(BlockHandle(new spb::KeepTopN(1)));

  return startBlock;
}

Factory::BlockHandle
Factory::createSearchPipeExtended(const OptionsMap & options) const
{
  // Create a search pipe
  BlockHandle startBlock = createSearchPipe(options);
  if(!startBlock)
    return NULL_HANDLE;

  // Are we doing a parameter sweep
  const OptionsMap * const paramSweepOptions = options.find(spf::SWEEP_POTENTIAL_PARAMS);
  if(paramSweepOptions)
  {
    BlockHandle sweepStartBlock;
    if(!myBlockFactory.createSweepPotentialParamsBlock(&sweepStartBlock, *paramSweepOptions, startBlock))
      return NULL_HANDLE;

    startBlock = sweepStartBlock;
  }
  return startBlock;
}

} // namespace stools
} // namespace factory
