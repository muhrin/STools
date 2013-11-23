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

#include <armadillo>

#include "common/StructureData.h"

// FORWARD DECLARES ////////////////////////////////

// DEFINES ////////////////////////////////////////
#define EPSILON_REL 1e-5;

// FUNCTIONS ////////////////////////////////////////

namespace spipe {
namespace common {

void
parseParamString(const std::string & str, double & from, double & step,
    unsigned int & nSteps);

::std::string
toString(const double in, unsigned int digitsAfterDecimal = 5);

::std::string
generateParamDirName(const ::arma::vec & params,
    const ::std::string & seedName);


}
}

#endif /* UTILITY_FUNCTIONS_H */
