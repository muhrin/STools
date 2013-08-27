/*
 * Random.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/math/Random.h"

// NAMESPACES ////////////////////////////////

namespace spl {
namespace math {
namespace detail {
::boost::mt19937 mt19937;

::boost::normal_distribution<> Rand<double>::normal(0.0, 1.0);
#ifdef SSLIB_USE_BOOST_OLD_RANDOM
const ::boost::uniform_real<> Rand<double>::uniform(0.0, 1.0);
::boost::variate_generator< ::boost::mt19937 &, ::boost::uniform_real<> >
  Rand<double>::uniformGenerator(mt19937, uniform);
boost::variate_generator< ::boost::mt19937 &, ::boost::normal_distribution<> >
  Rand<double>::normalGenerator(mt19937, normal);
#else
const ::boost::random::uniform_real_distribution<> Rand<double>::uniform(0.0, 1.0);
#endif

}
}
}

