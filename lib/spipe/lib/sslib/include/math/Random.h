/*
 * Random.h
 *
 *  Created on: Aug 22, 2011
 *      Author: Martin Uhrin
 */


#ifndef RANDOM_H
#define RANDOM_H

// INCLUDES ///////////////////////////////////////
#include <armadillo>

// FORWARD DECLARES ////////////////////////////////

// DEFINES ////////////////////////////////////////

// FUNCTIONS ////////////////////////////////////////

namespace sstbx {
namespace math {

void seed(const unsigned int);

template <typename T>
T randu();

template <typename T>
T randu(const T to);

template <typename T>
T randu(const T from, const T to);

template <typename T>
T randn();

template <typename T>
T randn(const T mean, const T variance);

}
}

#include "math/detail/Random.h"

#endif /* RANDOM_H */
