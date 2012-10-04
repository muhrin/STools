/*
 * DistanceCalculator.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES ///////////////
#include "common/DistanceCalculator.h"

#include <boost/assert.hpp>


namespace sstbx {
namespace common {

const size_t DistanceCalculator::DEFAULT_MAX_OUTPUTS = 10000;
const unsigned int DistanceCalculator::DEFAULT_MAX_CELL_MULTIPLES = 10000;

DistanceCalculator::DistanceCalculator(const Structure & structure):
myStructure(structure)
{}

}
}
