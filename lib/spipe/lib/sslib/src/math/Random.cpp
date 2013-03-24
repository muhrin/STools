/*
 * Random.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "math/Random.h"

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace math {
namespace detail {
boost::random::mt19937 mt19937;

const ::boost::random::uniform_real_distribution<> Rand<double>::uniform(0.0, 1.0);
::boost::random::normal_distribution<> Rand<double>::normal(0.0, 1.0);

}
}
}

