/*
 * TranscodeGeneral.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_GENERAL_H
#define TRANSCODE_GENERAL_H

// INCLUDES //////////////////////////////////
#include "SSLib.h"

#ifdef SSLIB_USE_YAML

#include <armadillo>

#include <yaml-cpp/yaml.h>

#include "yaml/HelperTypes.h"

// NAMESPACES ////////////////////////////////

// Some custom YAML transcoders
namespace YAML {

// Vector as string
template <typename T>
struct convert< ::sstbx::yaml::VectorAsString<T> >
{
  static Node encode(const ::sstbx::yaml::VectorAsString<T> & vector);
  static bool decode(const Node & node, ::sstbx::yaml::VectorAsString<T> & vector);
};

// Armadillo vec3
template<>
struct convert<arma::vec3>
{
  static Node encode(const arma::vec3 & rhs);
  static bool decode(const Node & node, arma::vec3 & rhs);
};

// Armadillo vec
template<>
struct convert<arma::vec>
{
  static Node encode(const arma::vec & rhs);
  static bool decode(const Node& node, arma::vec & rhs);
};

template<>
struct convert< ::sstbx::yaml::ArmaTriangularMat>
{
  static Node encode(const ::sstbx::yaml::ArmaTriangularMat & rhs);
  static bool decode(const Node & node, ::sstbx::yaml::ArmaTriangularMat & rhs);
};


}

#include "yaml/detail/TranscodeGeneral.h"

#endif /* SSLIB_USE_YAML */

#endif /* TRANSCODE_GENERAL_H */
