/*
 * BoostCapabilities.h
 *
 *
 *  Created on: Nov 19, 2012
 *      Author: Martin Uhrin
 */

#ifndef BOOST_CAPABILITIES_H
#define BOOST_CAPABILITIES_H

// INCLUDES /////////////////////////////////////////////
#include <boost/version.hpp>

// FORWARD DECLARES ////////////////////////////////

// DEFINES /////////////////////////////////////////

#if (BOOST_VERSION / 100000) > 1 
#  if ((BOOST_VERSION / 100) % 1000) >= 42
#   define BOOST_PROGRAM_OPTIONS_HAS_REQUIRED
#  endif
#endif

#if (BOOST_VERSION / 100000) > 1 && ((BOOST_VERSION / 100) % 1000) >= 42
# define _ADD_REQUIRED_ ->required()
#else
# define _ADD_REQUIRED_
#endif

#if (BOOST_VERSION / 100000) <= 1 && ((BOOST_VERSION / 100) % 1000) <= 45
#  define STOOLS_USE_BOOSTFS_V2
#endif

#endif /* BOOST_CAPABILITIES_H */
