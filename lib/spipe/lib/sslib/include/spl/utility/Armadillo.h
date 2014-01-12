/*
 * Armadillo.h
 *
 *
 *  Created on: Nov 1, 2013
 *      Author: Martin Uhrin
 */

#ifndef ARMADILLO_H
#define ARMADILLO_H

// INCLUDES /////////////////////////////////////////////
#include <boost/functional/hash_fwd.hpp>

#include <armadillo>

namespace arma {
::std::size_t
hash_value(const vec & v);

template< typename T>
  std::istream &
  operator >>(std::istream &in, arma::Mat< T> & mat)
  {
    mat.quiet_load(in);
    return in;
  }

}

#endif /* ARMADILLO_H */
