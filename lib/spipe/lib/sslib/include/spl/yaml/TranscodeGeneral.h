/*
 * TranscodeGeneral.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_GENERAL_H
#define TRANSCODE_GENERAL_H

// INCLUDES //////////////////////////////////
#include "spl/SSLib.h"

#ifdef SSLIB_USE_YAML

#include <boost/regex.hpp>

#include <armadillo>

#include <yaml-cpp/yaml.h>

#include "spl/utility/Range.h"
#include "spl/yaml/HelperTypes.h"

// NAMESPACES ////////////////////////////////

// Some custom YAML transcoders
namespace YAML {

// Vector as string
template <typename T>
struct convert< ::spl::yaml::VectorAsString<T> >
{
  static Node encode(const ::spl::yaml::VectorAsString<T> & vector);
  static bool decode(const Node & node, ::spl::yaml::VectorAsString<T> & vector);
};

// Armadillo fixed vectors
template<unsigned int size>
struct convert<arma::vec::fixed<size> >
{
  static Node encode(const arma::vec::fixed<size> & rhs);
  static bool decode(const Node & node, arma::vec::fixed<size> & rhs);
};

// Armadillo vec
template<>
struct convert<arma::vec>
{
  static Node encode(const arma::vec & rhs);
  static bool decode(const Node& node, arma::vec & rhs);
};

template<>
struct convert< ::spl::yaml::ArmaTriangularMat>
{
  static Node encode(const ::spl::yaml::ArmaTriangularMat & rhs);
  static bool decode(const Node & node, ::spl::yaml::ArmaTriangularMat & rhs);
};

template <typename T>
struct convert< ::spl::utility::Range<T> >
{
  static Node encode(const ::spl::utility::Range<T> & rhs);
  static bool decode(const Node & node, ::spl::utility::Range<T> & rhs);
};

}

#include "spl/yaml/detail/TranscodeGeneral.h"

#endif /* SSLIB_USE_YAML */

#endif /* TRANSCODE_GENERAL_H */
