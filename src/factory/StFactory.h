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

// From SSTbx
#include <factory/SsLibFactoryYaml.h>
#include <utility/HeterogeneousMap.h>

// From SPipe
#include <SpTypes.h>
#include <factory/Factory.h>

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx {
namespace common {
class AtomSpeciesDatabase;
}
}

namespace stools {
namespace factory {

class Factory
{
public:
  typedef ::sstbx::UniquePtr< ::spipe::SpBlock>::Type BlockPtr;
  typedef ::sstbx::UniquePtr< ::spipe::SpPipe>::Type PipePtr;
  typedef ::sstbx::utility::HeterogeneousMap OptionsMap;

  Factory(::sstbx::common::AtomSpeciesDatabase & speciesDb):
    mySsLibFactory(speciesDb),
    mySpFactory(speciesDb)
  {}

  bool createBuildPipe(PipePtr & pipeOut, const OptionsMap & options) const;
  bool createSearchPipe(PipePtr & pipeOut, const OptionsMap & options) const;
  bool createSearchPipeExtended(PipePtr & pipeOut, const OptionsMap & options) const;

  bool createGeomOptimiseBlock(
    BlockPtr & blockOut,
    const OptionsMap & geomOptimiseOptions
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
  ::sstbx::factory::SsLibFactoryYaml mySsLibFactory;

};



}
}

#endif /* STOOLS__FACTORY__FACTORY_H */

