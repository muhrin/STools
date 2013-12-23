/*
 * BlockFactory.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef BLOCK_FACTORY_H
#define BLOCK_FACTORY_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <map>
#include <string>

#include <spl/SSLib.h>
#include <spl/factory/SsLibFactoryYaml.h>
#include <spl/potential/OptimisationSettings.h>
#include <spl/utility/HeterogeneousMap.h>

// Local includes
#include "StructurePipe.h"
#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace spl {
namespace common {
class AtomSpeciesDatabase;
}
}

namespace spipe {
namespace factory {

class BlockFactory
{
public:
  typedef ::spl::utility::HeterogeneousMap OptionsMap;

  struct Blocks
  {
    enum Value
    {
      BUILD_STRUCTURES,
      CLONE_BLOCK,
      CUT_AND_PASTE,
      FIND_SYMMETRY,
#ifdef SSLIB_USE_CGAL
      KEEP_STABLE_COMPOSITIONS,
#endif
      KEEP_TOP_N,
      KEEP_WITHIN_X_PERCENT,
      LOAD_STRUCTURES,
      NIGGLI_REDUCE,
      GEOM_OPTIMISE,
      REMOVE_DUPLICATES,
      RUN_POTENTIAL_PARAMS_QUEUE,
      WRITE_STRUCTURES
    };
  };

  BlockFactory(::spl::common::AtomSpeciesDatabase & speciesDb);

  bool
  createBlock(const Blocks::Value block, const OptionsMap & options,
      BlockHandle * const blockOut) const;

  bool
  createBuildStructuresBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createCloneBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createCutAndPasteBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createFindSymmetryGroupBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
#ifdef SSLIB_USE_CGAL
  bool
  createKeepStableCompositionsBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
#endif
  bool
  createKeepTopNBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createKeepWithinXPercentBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createLoadStructuresBlock(BlockHandle * const blockOut,
      const ::std::string & toLoad) const;
  bool
  createNiggliReduceBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createGeomOptimiseBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createRemoveDuplicatesBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createRunPotentialParamsQueueBlock(BlockHandle * const blockOut,
      const OptionsMap & options, BlockHandle subpipe) const;
  bool
  createSearchStoichiometriesBlock(BlockHandle * const blockOut,
      const OptionsMap & options, BlockHandle subpipe) const;
  bool
  createSweepPotentialParamsBlock(BlockHandle * const blockOut,
      const OptionsMap & options, BlockHandle sweepPipe) const;
  bool
  createWriteStructuresBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;

private:
  ::spl::factory::Factory mySplFactory;
};

}
}

#endif /* BLOCK_FACTORY_H */

