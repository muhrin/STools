/*
 * UtilFunctions.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef UTIL_FUNCTIONS_H
#define UTIL_FUNCTIONS_H

// INCLUDES /////////////////////////////////////////////

#include <string>

// FORWARD DECLARES ////////////////////////////////

// DEFINES ////////////////////////////////////////

// FUNCTIONS ////////////////////////////////////////

namespace sstbx {
namespace utility {

  
std::string generateUniqueName(const size_t randPostfixLength = 4);

}
}

#endif /* UTIL_FUNCTIONS_H */
