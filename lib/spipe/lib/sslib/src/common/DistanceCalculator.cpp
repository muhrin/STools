/*
 * DistanceCalculator.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES ///////////////
#include "spl/common/DistanceCalculator.h"

namespace spl {
namespace common {

const size_t DistanceCalculator::DEFAULT_MAX_OUTPUTS = 10000;
// The maximum number of cell multiples to consider in each cell vector direction
const unsigned int DistanceCalculator::DEFAULT_MAX_CELL_MULTIPLES = 1000;

DistanceCalculator::DistanceCalculator(Structure & structure) :
    myStructure(structure)
{
}

}
}
