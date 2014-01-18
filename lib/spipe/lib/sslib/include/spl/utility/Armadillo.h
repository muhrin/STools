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
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL
#  include <CGAL/Point_2.h>
#endif

#include <boost/functional/hash_fwd.hpp>

#include <armadillo>

namespace spl {
namespace utility {

#ifdef SPL_WITH_CGAL

template <typename K>
CGAL::Point_2<K>
toCgalPoint(const arma::vec2 & r)
{
  return CGAL::Point_2<K>(r(0), r(1));
}


#endif


}
}

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
