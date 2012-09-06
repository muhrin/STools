/*
 * SSLibAssert.h
 *
 *  Created on: Sep 5, 2011
 *      Author: Martin Uhrin
 */

#ifndef SSLIB_ASSERT_H
#define SSLIB_ASSERT_H

// INCLUDES ///////////////////////////////////////////////

#include <boost/assert.hpp>

// DEFINES ///////////////////////////////////////////////

#define SSLIB_ASSERT(expr) BOOST_ASSERT(expr)

#if (BOOST_VERSION / 100000) <= 1 && ((BOOST_VERSION / 100) % 1000) <= 45
  // No BOOST_ASSERT_MSG was introduced in 1.46
#  define SSLIB_ASSERT_MSG(expr, msg) BOOST_ASSERT(expr)
#else
#  define SSLIB_ASSERT_MSG(expr, msg) BOOST_ASSERT_MSG(expr, msg)
#endif

#endif /* SSLIB_ASSERT_H */
