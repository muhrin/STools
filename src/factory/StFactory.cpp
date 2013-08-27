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
#include <blocks/LowestFreeEnergy.h>

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

bool Factory::createBuildPipe(PipePtr & pipeOut, const OptionsMap & options) const
{
  const OptionsMap * const randomStructureOptions = options.find(spf::RANDOM_STRUCTURE);

  if(!randomStructureOptions)
    return false;

  spf::Factory::BlockPtr block;
  if(!mySpFactory.createRandomStructureBlock(block, *randomStructureOptions))
    return false;

  PipePtr pipe(new sp::SpPipe());

  // Keep track of the last block so we can connect everything up
  sp::SpBlock * lastBlock = pipe->addBlock(block.release());
  pipe->setStartBlock(lastBlock->asStartBlock());

  const OptionsMap * const outputOptions = options.find(spf::WRITE_STRUCTURES);
  if(outputOptions)
  {
    if(mySpFactory.createWriteStructuresBlock(block, *outputOptions))
    {
      lastBlock = addAndConnect(*pipe, lastBlock, block.release());
    }
    else
      return false;
  }

  pipeOut = pipe;
  return true;
}

bool
Factory::createSearchPipe(PipePtr & pipeOut, const OptionsMap & options) const
{
  const OptionsMap * const randomStructureOptions = options.find(spf::RANDOM_STRUCTURE);

  if(!randomStructureOptions)
    return false;

  spf::Factory::BlockPtr block;
  if(!mySpFactory.createRandomStructureBlock(block, *randomStructureOptions))
    return false;

  PipePtr pipe(new sp::SpPipe());

  // Keep track of the last block so we can connect everything up
  sp::SpBlock * lastBlock = pipe->addBlock(block.release());
  pipe->setStartBlock(lastBlock->asStartBlock());

  const OptionsMap * const preGeomOptimiseOptions = options.find(spf::PRE_GEOM_OPTIMISE);
  if(preGeomOptimiseOptions)
  {
    if(createGeomOptimiseBlock(block, *preGeomOptimiseOptions, &options))
    {
      lastBlock = addAndConnect(*pipe, lastBlock, block.release());
    }
    else
      return false;
  }

  const OptionsMap * const geomOptimiseOptions = options.find(spf::GEOM_OPTIMISE);
  if(geomOptimiseOptions)
  {
    if(createGeomOptimiseBlock(block, *geomOptimiseOptions, &options))
    {
      lastBlock = addAndConnect(*pipe, lastBlock, block.release());
    }
    else
      return false;
  }

  const OptionsMap * const removeDuplicatesOptions = options.find(spf::REMOVE_DUPLICATES);
  if(removeDuplicatesOptions)
  {
    if(mySpFactory.createRemoveDuplicatesBlock(block, *removeDuplicatesOptions))
      lastBlock = addAndConnect(*pipe, lastBlock, block.release());
    else
      return false;
  }

  // Find out what the symmetry group is
  if(mySpFactory.createDetermineSpaceGroupBlock(block))
    lastBlock = addAndConnect(*pipe, lastBlock, block.release());
  else
    return false;

  const OptionsMap * const lowestEnergyOptions = options.find(spf::LOWEST_ENERGY);
  if(lowestEnergyOptions)
  {
    if(mySpFactory.createLowestEnergyBlock(block, *lowestEnergyOptions))
      lastBlock = addAndConnect(*pipe, lastBlock, block.release());
    else
      return false;
  }

  const OptionsMap * const outputOptions = options.find(spf::WRITE_STRUCTURES);
  if(outputOptions)
  {
    lastBlock = addWriteStructuresBlock(*pipe, lastBlock, *outputOptions, true);
  }

  // Finally tack on a lowest energy block to make sure that only one structure
  // comes out the end in all eventualities
  lastBlock = addAndConnect(*pipe, lastBlock, new spb::LowestFreeEnergy(static_cast<size_t>(1)));

  pipeOut = pipe;
  return true;
}

bool Factory::createSearchPipeExtended(PipePtr & pipeOut, const OptionsMap & options) const
{
  const OptionsMap * const paramSweepOptions = options.find(spf::PARAM_SWEEP);

  // Create a search pipe
  PipePtr searchPipe;
  if(!createSearchPipe(searchPipe, options))
    return false;

  // Are we doing a parameter sweep
  if(paramSweepOptions)
  {
    // Create the param sweep pipe
    PipePtr paramSweepPipe(new sp::SpPipe());

    spf::Factory::BlockPtr block;
    // Try creating the parameter sweep block
    if(!mySpFactory.createParamSweepBlock(block, *paramSweepOptions, searchPipe))
      return false;

    // Keep track of the last block so we can connect everything up
    sp::SpBlock * lastBlock = paramSweepPipe->addBlock(block.release());
    paramSweepPipe->setStartBlock(lastBlock->asStartBlock());
    
    pipeOut = paramSweepPipe;
  }
  else
    pipeOut = searchPipe; // Only doing search, so search pipe is the 'main' pipe

  return true;
}

bool Factory::createGeomOptimiseBlock(
  BlockPtr & blockOut,
  const OptionsMap & geomOptimiseOptions,
  const OptionsMap * globalOptions
) const
{
  // Try to find the optimiser options
  const OptionsMap * const optimiserOptions = geomOptimiseOptions.find(ssf::OPTIMISER);
  if(!optimiserOptions)
    return false;

  // Try to find a potential options
  const OptionsMap * const potentialOptions = geomOptimiseOptions.find(ssf::POTENTIAL);

  // Set up the optimisation settings
  ssp::OptimisationSettings optimisationSettings;
  const double * const pressure = geomOptimiseOptions.find(ssf::PRESSURE);
  if(pressure)
  {
    ::arma::mat33 pressureMtx = ::arma::zeros< ::arma::mat>(3,3);
    pressureMtx.diag().fill(*pressure);
    optimisationSettings.pressure.reset(pressureMtx);
  }

  return mySpFactory.createPotentialGeomOptimiseBlock(
    blockOut,
    *optimiserOptions,
    potentialOptions,
    &optimisationSettings,
    globalOptions
  ); 
}

::spipe::SpBlock * Factory::addWriteStructuresBlock(
  ::spipe::SpPipe & pipe,
  ::spipe::SpBlock * lastBlock,
  const OptionsMap & outputOptions,
  const bool useBarrier
) const
{
  spf::Factory::BlockPtr block;
  if(mySpFactory.createWriteStructuresBlock(block, outputOptions))
  {
    lastBlock = addAndConnect(pipe, lastBlock, block.release());
    if(useBarrier)
    {
      lastBlock = addAndConnect(pipe, lastBlock, new sp::SpSimpleBarrier());
      // Create another writer immediately after the barrier
      mySpFactory.createWriteStructuresBlock(block, outputOptions);
      lastBlock = addAndConnect(pipe, lastBlock, block.release());
    }
  }

  return lastBlock;
}

sp::SpBlock *
Factory::addAndConnect(
  ::spipe::SpPipe & pipe,
  sp::SpBlock * const lastBlock,
  sp::SpBlock * const newBlock) const
{
  SSLIB_ASSERT_MSG(newBlock->isPipeBlock(), "Can only connect an input to a pipe block.");

  pipe.addBlock(newBlock);
  pipe.connect(lastBlock, newBlock->asPipeBlock());
  return newBlock;
}


} // namespace stools
} // namespace factory
