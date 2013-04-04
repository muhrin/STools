/*
 * TranscodeFactory.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_FACTORY_H
#define TRANSCODE_FACTORY_H

// INCLUDES //////////////////////////////////
#include "SSLib.h"

#ifdef SSLIB_USE_YAML

#include <yaml-cpp/yaml.h>

#include "factory/FactoryFwd.h"

// NAMESPACES ////////////////////////////////

// Some custom YAML transcoders
namespace YAML {

// factory::MinMax
template <>
struct convert < ::sstbx::factory::MinMax>
{
  static Node encode(const ::sstbx::factory::MinMax & rhs);
  static bool decode(const Node & node, ::sstbx::factory::MinMax & rhs);
};

// factory::AtomSpeciesCount
template <>
struct convert < ::sstbx::factory::AtomSpeciesCount>
{
  static Node encode(const ::sstbx::factory::AtomSpeciesCount & rhs);
  static bool decode(const Node & node, ::sstbx::factory::AtomSpeciesCount & rhs);
};



}

#endif /* SSLIB_USE_YAML */
#endif /* TRANSCODE_FACTORY_H */
