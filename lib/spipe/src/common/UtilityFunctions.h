/*
 * UtilityFunctions.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

// INCLUDES /////////////////////////////////////////////

#include <string>

// FORWARD DECLARES ////////////////////////////////
namespace spipe {
namespace common {
class StructureData;
}
}

// DEFINES ////////////////////////////////////////
#define EPSILON_REL 1e-5;

// FUNCTIONS ////////////////////////////////////////

namespace spipe {
namespace common {

void parseParamString(
  const std::string & str,
  double &            from,
  double &            step,
  unsigned int &      nSteps
);

::std::string getString(const double in, unsigned int digitsAfterDecimal = 5);

}
}

#endif /* UTILITY_FUNCTIONS_H */
