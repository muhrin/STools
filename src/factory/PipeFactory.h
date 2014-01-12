/*
 * PipeFactory.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef PIPE_FACTORY_H
#define PIPE_FACTORY_H

// INCLUDES /////////////////////////////////////////////
#include "STools.h"

#include <boost/optional.hpp>

#include <spl/factory/SsLibFactoryYaml.h>

// From SPipe
#include <SpTypes.h>
#include <blocks/KeepTopN.h>
#include <factory/BlockFactory.h>

// FORWARD DECLARATIONS ////////////////////////////////////
namespace spl {
namespace common {
class AtomSpeciesDatabase;
}
}

namespace stools {
namespace factory {

class PipeFactory
{
public:
  typedef ::spipe::BlockHandle BlockHandle;
  typedef ::spl::utility::HeterogeneousMap OptionsMap;

  PipeFactory(::spl::common::AtomSpeciesDatabase & speciesDb) :
      myBlockFactory(speciesDb), mySsLibFactory(speciesDb)
  {
  }

  template< class T>
    BlockHandle
    createBuildPipe(const T & options) const;
  template< class T>
    BlockHandle
    createSearchPipe(const T & options) const;
  template< class T>
    BlockHandle
    createSearchPipeExtended(const T & options) const;

private:
  ::spipe::factory::BlockFactory myBlockFactory;
  ::spl::factory::Factory mySsLibFactory;
};

template< class T>
  PipeFactory::BlockHandle
  PipeFactory::createBuildPipe(const T & options) const
  {
    if(!options.buildStructures)
      return BlockHandle();

    BlockHandle startBlock;
    if(!myBlockFactory.createBlock(&startBlock,
        *options.buildStructures))
      return BlockHandle();

    // Keep track of the last block so we can connect everything up
    BlockHandle block, lastBlock = startBlock;

    if(options.buildStructures)
    {
      if(myBlockFactory.createBlock(&block,
          *options.buildStructures))
        lastBlock = lastBlock->connect(block);
      else
        return BlockHandle();
    }

    return startBlock;
  }

template< class T>
  PipeFactory::BlockHandle
  PipeFactory::createSearchPipe(const T & options) const
  {
    {
      BlockHandle startBlock;
      if(options.buildStructures)
      {
        if(!myBlockFactory.createBlock(&startBlock,
            *options.buildStructures))
          return BlockHandle();
      }
      else if(options.loadStructures)
      {
        if(!myBlockFactory.createBlock(&startBlock,
            *options.loadStructures))
          return BlockHandle();
      }
      else
        return BlockHandle();

      // Keep track of the last block so we can connect everything up
      BlockHandle block, lastBlock = startBlock;

      if(options.cutAndPaste
          && myBlockFactory.createBlock(&block,
              *options.cutAndPaste))
        lastBlock = lastBlock->connect(block);

      if(options.preGeomOptimise
          && myBlockFactory.createBlock(&block,
              *options.preGeomOptimise))
        lastBlock = lastBlock->connect(block);

      if(options.geomOptimise
          && myBlockFactory.createBlock(&block,
              *options.geomOptimise))
        lastBlock = lastBlock->connect(block);

      if(options.removeDuplicates
          && myBlockFactory.createBlock(&block,
              *options.removeDuplicates))
        lastBlock = lastBlock->connect(block);

      if(options.keepWithinXPercent
          && myBlockFactory.createBlock(&block,
              *options.keepWithinXPercent))
        lastBlock = lastBlock->connect(block);

      if(options.keepTopN
          && myBlockFactory.createBlock(&block, *options.keepTopN))
        lastBlock = lastBlock->connect(block);

      // Find out what the symmetry group is
      if(options.findSymmetryGroup
          && myBlockFactory.createBlock(&block,
              *options.findSymmetryGroup))
        lastBlock = lastBlock->connect(block);

      if(options.writeStructures
          && myBlockFactory.createBlock(&block,
              *options.writeStructures))
        lastBlock = lastBlock->connect(block);

      // Finally tack on a lowest energy block to make sure that only one structure
      // comes out the end in all eventualities
      lastBlock = lastBlock->connect(
          BlockHandle(new ::spipe::blocks::KeepTopN(1)));

      return startBlock;
    }
  }

template< class T>
  PipeFactory::BlockHandle
  PipeFactory::createSearchPipeExtended(const T & options) const
  {
    // Create a search pipe
    BlockHandle startBlock = createSearchPipe(options);
    if(!startBlock)
      return BlockHandle();

    // Are we doing a stoichiometry search?
    if(options.searchStoichiometries)
    {
      BlockHandle searchStoichsBlock;
      if(!myBlockFactory.createBlock(&searchStoichsBlock,
          *options.searchStoichiometries, startBlock))
        return BlockHandle();

      startBlock = searchStoichsBlock;

#ifdef SPL_WITH_CGAL
      if(options.keepStableCompositions)
      {
        BlockHandle block;
        if(myBlockFactory.createBlock(&block,
            *options.keepStableCompositions))
          startBlock->connect(block);
      }
#endif
    }

    // Are we doing a parameter sweep
    if(options.sweepPotentialParams)
    {
      BlockHandle sweepStartBlock;
      if(!myBlockFactory.createBlock(&sweepStartBlock,
          *options.sweepPotentialParams, startBlock))
        return BlockHandle();

      return sweepStartBlock;
    }

    if(options.runPotParamsQueue)
    {
      BlockHandle runQueueBlock;
      if(!myBlockFactory.createBlock(&runQueueBlock,
          *options.runPotParamsQueue, startBlock))
        return BlockHandle();

      return runQueueBlock;
    }

    return startBlock;
  }

}
}

#endif /* PIPE_FACTORY_H */

