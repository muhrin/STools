/*
 * Random.h
 *
 *  Created on: Aug 22, 2011
 *      Author: Martin Uhrin
 */


#ifndef RANDOM_H
#define RANDOM_H

// INCLUDES ///////////////////////////////////////

// FORWARD DECLARES ////////////////////////////////

// DEFINES ////////////////////////////////////////

// FUNCTIONS ////////////////////////////////////////

namespace sstbx {
namespace math {

void seed(const unsigned int);

template <typename T>
T rand();

template <typename T>
T rand(const T to);

template <typename T>
T rand(const T from, const T to);

}
}

#include "math/detail/Random.h"

#endif /* RANDOM_H */
