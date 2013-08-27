/*
 * TranscodeFactory.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_FACTORY_H
#define TRANSCODE_FACTORY_H

// INCLUDES //////////////////////////////////
#include "spl/SSLib.h"

#ifdef SSLIB_USE_YAML

#include <yaml-cpp/yaml.h>

#include "spl/factory/FactoryFwd.h"

// NAMESPACES ////////////////////////////////

// Some custom YAML transcoders
namespace YAML {

// factory::MinMax
template <>
struct convert < ::spl::factory::MinMax>
{
  static Node encode(const ::spl::factory::MinMax & rhs);
  static bool decode(const Node & node, ::spl::factory::MinMax & rhs);
};

// factory::AtomSpeciesCount
template <>
struct convert < ::spl::factory::AtomSpeciesCount>
{
  static Node encode(const ::spl::factory::AtomSpeciesCount & rhs);
  static bool decode(const Node & node, ::spl::factory::AtomSpeciesCount & rhs);
};



}

#endif /* SSLIB_USE_YAML */
#endif /* TRANSCODE_FACTORY_H */
