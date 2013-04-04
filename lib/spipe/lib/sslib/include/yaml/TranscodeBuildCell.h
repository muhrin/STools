/*
 * YamlTranscode.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef YAML_TRANSCODE_H
#define YAML_TRANSCODE_H

// INCLUDES //////////////////////////////////
#include "SSLib.h"

#ifdef SSLIB_USE_YAML

#include <yaml-cpp/yaml.h>

#include "build_cell/Sphere.h"

// NAMESPACES ////////////////////////////////

// Some custom YAML transcoders
namespace YAML {

// build_cell::Sphere
template<>
struct convert< ::sstbx::build_cell::Sphere>
{
  static Node encode(const ::sstbx::build_cell::Sphere & sphere);
  static bool decode(const Node & node, ::sstbx::build_cell::Sphere & sphere);
};

}


#endif /* YAML_COMMON_H */

#endif /* YAML_TRANSCODE_H */
