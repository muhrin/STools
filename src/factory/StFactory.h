/*
 * Factory.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef STOOLS__FACTORY__FACTORY_H
#define STOOLS__FACTORY__FACTORY_H

// INCLUDES /////////////////////////////////////////////
#include "STools.h"

#include <boost/optional.hpp>

#include <spl/factory/SsLibFactoryYaml.h>
#include <spl/utility/HeterogeneousMap.h>

// From SPipe
#include <SpTypes.h>
#include <factory/Factory.h>

// FORWARD DECLARATIONS ////////////////////////////////////
namespace spl {
namespace common {
class AtomSpeciesDatabase;
}
}

namespace stools {
namespace factory {

class Factory
{
public:
  typedef ::spl::UniquePtr< ::spipe::SpBlock>::Type BlockPtr;
  typedef ::spl::UniquePtr< ::spipe::SpPipe>::Type PipePtr;
  typedef ::spl::utility::HeterogeneousMap OptionsMap;

  Factory(::spl::common::AtomSpeciesDatabase & speciesDb):
    mySsLibFactory(speciesDb),
    mySpFactory(speciesDb)
  {}

  bool createBuildPipe(PipePtr & pipeOut, const OptionsMap & options) const;
  bool createSearchPipe(PipePtr & pipeOut, const OptionsMap & options) const;
  bool createSearchPipeExtended(PipePtr & pipeOut, const OptionsMap & options) const;

  bool createGeomOptimiseBlock(
    BlockPtr & blockOut,
    const OptionsMap & geomOptimiseOptions,
    const OptionsMap * globalOptions = NULL
  ) const;

private:

  ::spipe::SpBlock * addWriteStructuresBlock(
    ::spipe::SpPipe & pipe,
    ::spipe::SpBlock * lastBlock,
    const OptionsMap & outputOptions,
    const bool useBarrier
  ) const;

  ::spipe::SpBlock * addAndConnect(
    ::spipe::SpPipe & pipe,
    ::spipe::SpBlock * const lastBlock,
    ::spipe::SpBlock * const newBlock
  ) const;

  ::spipe::factory::Factory mySpFactory;
  ::spl::factory::SsLibFactoryYaml mySsLibFactory;

};



}
}

#endif /* STOOLS__FACTORY__FACTORY_H */

