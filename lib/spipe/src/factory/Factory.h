/*
 * Factory.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SPIPE__FACTORY__FACTORY_H
#define SPIPE__FACTORY__FACTORY_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/optional.hpp>


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


class Factory
{
public:
  typedef ::spl::UniquePtr<SpBlock>::Type BlockPtr;
  typedef ::spl::UniquePtr<SpPipe>::Type PipePtr;
  typedef ::spl::utility::HeterogeneousMap OptionsMap;

  Factory(::spl::common::AtomSpeciesDatabase & speciesDb):
    mySsLibFactory(speciesDb)
  {}

  bool createDetermineSpaceGroupBlock(BlockPtr & blockOut) const;
  bool createLowestEnergyBlock(BlockPtr & blockOut, const OptionsMap & options) const;
  bool createNiggliReduceBlock(BlockPtr & blockOut) const;
  bool createParamPotentialGeomOptimiseBlock(BlockPtr & blockOut, const OptionsMap & options) const;
  bool createParamSweepBlock(BlockPtr & blockOut, const OptionsMap & options, PipePtr subPipe) const;
  bool createPotentialGeomOptimiseBlock(
    BlockPtr & blockOut,
    const OptionsMap & optimiserOptions,
    const OptionsMap * const potentialOptions = NULL,
    const ::spl::potential::OptimisationSettings * optimisationSettings = NULL,
    const OptionsMap * const globalOptions = NULL
  ) const;
  bool createRandomStructureBlock(BlockPtr & blockOut, const OptionsMap & options) const;
  bool createRemoveDuplicatesBlock(BlockPtr & blockOut, const OptionsMap & options) const;
  bool createWriteStructuresBlock(BlockPtr & blockOut, const OptionsMap & options) const;

private:

  ::spl::factory::SsLibFactoryYaml mySsLibFactory;

};


}
}

#endif /* SPIPE__FACTORY__FACTORY_H */

