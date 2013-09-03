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

#include <spl/potential/OptimisationSettings.h>
#include <spl/utility/HeterogeneousMap.h>

// Local includes
#include "StructurePipe.h"
#include "SpTypes.h"
#include "spl/factory/SsLibFactoryYaml.h"

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

  BlockFactory(::spl::common::AtomSpeciesDatabase & speciesDb) :
      mySplFactory(speciesDb)
  {
  }

  bool
  createBuildStructuresBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createFindSymmetryGroupBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createKeepTopNBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
  bool
  createKeepWithinXPercentBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;
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
  createSweepPotentialParamsBlock(BlockHandle * const blockOut,
      const OptionsMap & options, BlockHandle sweepPipe) const;
  bool
  createWriteStructuresBlock(BlockHandle * const blockOut,
      const OptionsMap & options) const;

private:
  ::spl::factory::SsLibFactoryYaml mySplFactory;
};

}
}

#endif /* BLOCK_FACTORY_H */

