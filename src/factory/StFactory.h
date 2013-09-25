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
#include <factory/BlockFactory.h>

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
  typedef ::spipe::BlockHandle BlockHandle;
  typedef ::spl::utility::HeterogeneousMap OptionsMap;

  Factory(::spl::common::AtomSpeciesDatabase & speciesDb):
    myBlockFactory(speciesDb),
    mySsLibFactory(speciesDb)
  {}

  BlockHandle createBuildPipe(const OptionsMap & options) const;
  BlockHandle createSearchPipe(const OptionsMap & options) const;
  BlockHandle createSearchPipeExtended(const OptionsMap & options) const;

private:
  static const BlockHandle NULL_HANDLE;

  ::spipe::factory::BlockFactory myBlockFactory;
  ::spl::factory::SsLibFactoryYaml mySsLibFactory;
};



}
}

#endif /* STOOLS__FACTORY__FACTORY_H */

