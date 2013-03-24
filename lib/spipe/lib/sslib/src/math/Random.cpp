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
::boost::mt19937 mt19937;

::boost::normal_distribution<> Rand<double>::normal(0.0, 1.0);
#ifdef SSLIB_USE_BOOST_OLD_RANDOM
const ::boost::random::uniform_real<> Rand<double>::uniform(0.0, 1.0);
#else
const ::boost::random::uniform_real_distribution<> Rand<double>::uniform(0.0, 1.0);
#endif

}
}
}

