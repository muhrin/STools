/*
 * TerminalFunctions.h
 *
 *
 *  Created on: Nov 19, 2012
 *      Author: Martin Uhrin
 */

#ifndef TERMINAL_FUNCTIONS_H
#define TERMINAL_FUNCTIONS_H

// INCLUDES /////////////////////////////////////////////
#include <ostream>

// FORWARD DECLARES ////////////////////////////////
namespace stools {
namespace utility {

bool
isStdInPipedOrFile();

std::ostream &
warning();

std::ostream &
error();

std::ostream &
info();

}
}

#endif /* TERMINAL_FUNCTIONS_H */
