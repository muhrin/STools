/*
 * TerminalFunctions.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "TerminalFunctions.h"

#include <stdio.h>

#ifdef WIN32
#  include <io.h>
#endif

#ifdef __unix__
#  include <unistd.h>
#endif

// NAMESPACES ////////////////////////////////

namespace stools {
namespace utility {

bool isStdInPipedOrFile()
{
#ifdef WIN32
  return !_isatty(_fileno(stdin));
#else ifdef __unix__
  return !isatty(fileno(stdin));
#endif
}


}
}
