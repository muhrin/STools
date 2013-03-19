/*
 * SymmetryFunctions.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SYMMETRY_FUNCTIONS_H
#define SYMMETRY_FUNCTIONS_H

// INCLUDES /////////////////////////////////
#include "SSLib.h"

#include <vector>

// FORWARD DECLARES //////////////////////////

namespace sstbx {
namespace build_cell {
namespace symmetry {

::std::vector<unsigned int>
generateMultiplicities(const unsigned int numAtoms, const unsigned int numSymOps);

::std::vector<unsigned int>
generateMultiplicities(const unsigned int numAtoms, const ::std::vector<unsigned int>  & possibleMultiplicities);

}
}
}

#endif /* SYMMETRY_FUNCTIONS_H */
