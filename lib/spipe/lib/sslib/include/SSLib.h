/*
 * SSLib.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SSLIB_H
#define SSLIB_H

// INCLUDES ///////////////////////////////////////////////

#include "SSLibConfig.h"

#include "SSLibTypes.h"

// DEFINES ///////////////////////////////////////////////
#ifndef NULL
  #define NULL 0
#endif

#define SSLIB_DEBUG 1

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
#  define SSLIB_OS_POSIX
#endif


#endif /* SSLIB_H */
