/*
 * TranscodeCommon.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_COMMON_H
#define TRANSCODE_COMMON_H

// INCLUDES //////////////////////////////////
#include "spl/SSLib.h"

#ifdef SSLIB_USE_YAML

#include <yaml-cpp/yaml.h>

#include "spl/common/UnitCell.h"

// NAMESPACES ////////////////////////////////

// Some custom YAML transcoders
namespace YAML {

// common::UnitCell
template<>
struct convert< ::spl::common::UnitCell>
{
  static Node encode(const ::spl::common::UnitCell & cell);
  static bool decode(const Node & node, ::spl::common::UnitCell & cell);
};

}

#endif /* SSLIB_USE_YAML */
#endif /* TRANSCODE_COMMON_H */
