/*
 * Assert.h
 *
 *  Created on: Sep 25, 2013
 *      Author: Martin Uhrin
 */

#ifndef SPIPE_ASSERT_H
#define SPIPE_ASSERT_H

// INCLUDES ///////////////////////////////////////////////
#include <boost/assert.hpp>
#include <boost/version.hpp>

// DEFINES ///////////////////////////////////////////////

#define SPIPE_ASSERT(expr) BOOST_ASSERT(expr)

#if (BOOST_VERSION / 100000) <= 1 && ((BOOST_VERSION / 100) % 1000) <= 45
  // No BOOST_ASSERT_MSG.  Was introduced in 1.46
#  define SPIPE_ASSERT_MSG(expr, msg) BOOST_ASSERT(expr)
#else
#  define SPIPE_ASSERT_MSG(expr, msg) BOOST_ASSERT_MSG(expr, msg)
#endif


#endif /* SPIPE_ASSERT_H */
