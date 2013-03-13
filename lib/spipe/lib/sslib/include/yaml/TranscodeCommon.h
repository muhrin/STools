/*
 * TranscodeCommon.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_COMMON_H
#define TRANSCODE_COMMON_H

// INCLUDES //////////////////////////////////
#include "SSLib.h"

#ifdef SSLIB_USE_YAML

#include <yaml-cpp/yaml.h>

#include "common/UnitCell.h"

// NAMESPACES ////////////////////////////////

// Some custom YAML transcoders
namespace YAML {

// common::UnitCell
template<>
struct convert< ::sstbx::common::UnitCell>
{
  static Node encode(const ::sstbx::common::UnitCell & cell);
  static bool decode(const Node & node, ::sstbx::common::UnitCell & cell);
};

}

#endif /* SSLIB_USE_YAML */
#endif /* TRANSCODE_COMMON_H */
