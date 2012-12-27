/*
 * SharedData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/CommonData.h"


// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace ssu = ::sstbx::utility;

ParamRange::ParamRange(const size_t dims):
from(dims),
step(dims),
nSteps(dims)
{}

// Objects keys ////////////////
ssu::Key< ::arma::vec> GlobalKeys::POTENTIAL_PARAMS;
ssu::Key<ParamRange> GlobalKeys::POTENTIAL_SWEEP_RANGE;


}
}
