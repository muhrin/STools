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

#ifdef SPL_WITH_YAML

#include <armadillo>

#include <yaml-cpp/yaml.h>

#include "spl/utility/Range.h"
#include "spl/yaml/HelperTypes.h"

// NAMESPACES ////////////////////////////////

// Some custom YAML transcoders
namespace YAML {

// Vector as string
template< typename T>
  struct convert< ::spl::yaml::TypeWrapper< ::std::vector< T> > >
  {
    static Node
    encode(const typename ::spl::yaml::VecAsString< T>::Type & vector);
    static bool
    decode(const Node & node,
        typename ::spl::yaml::VecAsString< T>::Type & vector);
  };

// Armadillo fixed vectors
template< unsigned int size>
  struct convert< arma::vec::fixed< size> >
  {
    static Node
    encode(const arma::vec::fixed< size> & rhs);
    static bool
    decode(const Node & node, arma::vec::fixed< size> & rhs);
  };

// Armadillo vec
template< >
  struct convert< arma::vec>
  {
    static Node
    encode(const arma::vec & rhs);
    static bool
    decode(const Node& node, arma::vec & rhs);
  };

template< >
  struct convert< ::spl::yaml::ArmaTriangularMat>
  {
    static Node
    encode(const ::spl::yaml::ArmaTriangularMat & rhs);
    static bool
    decode(const Node & node, ::spl::yaml::ArmaTriangularMat & rhs);
  };

template< typename T>
  struct convert< ::spl::utility::Range< T> >
  {
    static Node
    encode(const ::spl::utility::Range< T> & rhs);
    static bool
    decode(const Node & node, ::spl::utility::Range< T> & rhs);
  };

}

#include "spl/yaml/detail/TranscodeGeneral.h"

#endif /* SPL_WITH_YAML */

#endif /* TRANSCODE_GENERAL_H */
