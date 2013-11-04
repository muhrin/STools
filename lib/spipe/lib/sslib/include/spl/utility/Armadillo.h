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

namespace spl {
namespace utility {

}
}

namespace arma {
::std::size_t hash_value(const vec & v);
}

#endif /* ARMADILLO_H */
