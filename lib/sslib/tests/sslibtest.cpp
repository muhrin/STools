/*
 * sslibtest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////

#include "sslibtest.h"

#ifdef SSLIB_BOOST_DYN_LINK
#define BOOST_TEST_DYN_LINK
#endif

// Initialise the boost testing framework
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
