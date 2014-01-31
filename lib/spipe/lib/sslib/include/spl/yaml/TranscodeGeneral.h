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
  struct convert< spl::yaml::TypeWrapper< std::vector< T> > >
  {
    static Node
    encode(const typename spl::yaml::VecAsString< T>::Type & vector);
    static bool
    decode(const Node & node,
        typename spl::yaml::VecAsString< T>::Type & vector);
  };

// Armadillo
template< typename T>
  struct convert< arma::Mat< T> >
  {
    static Node
    encode(const arma::Mat< T> & rhs);
    static bool
    decode(const Node & node, arma::Mat< T> & rhs);
  };

template< typename T>
  struct convert< arma::Col< T> >
  {
    static Node
    encode(const arma::Col< T> & rhs)
    {
      return convert< arma::Mat< T> >::encode(rhs);
    }
    static bool
    decode(const Node & node, arma::Col< T> & rhs)
    {
      return convert< arma::Mat< T> >::decode(node, rhs);
    }
  };

template< typename arma::uword fixed_n_elem>
  struct convert< arma::vec::fixed< fixed_n_elem> >
  {
    static Node
    encode(const arma::vec::fixed< fixed_n_elem> & rhs)
    {
      return convert< arma::mat>::encode(rhs);
    }
    static bool
    decode(const Node & node, arma::vec::fixed< fixed_n_elem> & rhs)
    {
      return convert< arma::mat>::decode(node, rhs);
    }
  };

template< typename arma::uword fixed_n_elem>
  struct convert< arma::rowvec::fixed< fixed_n_elem> >
  {
    static Node
    encode(const arma::rowvec::fixed< fixed_n_elem> & rhs)
    {
      return convert< arma::mat>::encode(rhs);
    }
    static bool
    decode(const Node & node, arma::rowvec::fixed< fixed_n_elem> & rhs)
    {
      return convert< arma::mat>::decode(node, rhs);
    }
  };

template< typename arma::uword n_rows, typename arma::uword n_cols>
  struct convert< arma::mat::fixed< n_rows, n_cols> >
  {
    static Node
    encode(const arma::mat::fixed< n_rows, n_cols> & rhs)
    {
      return convert< arma::mat>::encode(rhs);
    }
    static bool
    decode(const Node & node, arma::mat::fixed< n_rows, n_cols> & rhs)
    {
      return convert< arma::mat>::decode(node, rhs);
    }
  };

template< >
  struct convert< spl::yaml::ArmaTriangularMat>
  {
    static Node
    encode(const spl::yaml::ArmaTriangularMat & rhs);
    static bool
    decode(const Node & node, spl::yaml::ArmaTriangularMat & rhs);
  };

template< typename T>
  struct convert< spl::utility::OrderedPair< T> >
  {
    static Node
    encode(const spl::utility::OrderedPair< T> & rhs);
    static bool
    decode(const Node & node, spl::utility::OrderedPair< T> & rhs);
  };

template< typename T>
  struct convert< spl::utility::Range< T> >
  {
    static Node
    encode(const spl::utility::Range< T> & rhs);
    static bool
    decode(const Node & node, spl::utility::Range< T> & rhs);
  };

}

#include "spl/yaml/detail/TranscodeGeneral.h"

#endif /* SPL_WITH_YAML */
#endif /* TRANSCODE_GENERAL_H */
