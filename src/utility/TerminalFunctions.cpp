/*
 * TerminalFunctions.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/TerminalFunctions.h"

#include <stdio.h>
#include <iostream>

#ifdef WIN32
#  include <io.h>
#endif

#ifdef __unix__
#  include <unistd.h>
#endif

// NAMESPACES ////////////////////////////////

namespace stools {
namespace utility {

bool
isStdInPipedOrFile()
{
#ifdef WIN32
  return !_isatty(_fileno(stdin));
#endif
#ifdef __unix__
  return !isatty(fileno(stdin));
#endif
}

std::ostream &
warning()
{
  std::cerr << "Warning: ";
  return std::cerr;
}

std::ostream &
error()
{
  std::cerr << "Error: ";
  return std::cerr;
}

std::ostream &
info()
{
  return std::cout;
}

}
}
