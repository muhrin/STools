/*
 * CommonData.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef COMMON_DATA_H
#define COMMON_DATA_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <armadillo>

#include <utility/HeterogeneousMap.h>

// FORWARD DECLARATIONS ////////////////////////////////////


namespace spipe {
namespace common {

struct ParamRange
{
  ParamRange() {}
  ParamRange(const size_t dims);
  ::arma::vec from;
  ::arma::vec step;
  ::arma::Col<unsigned int> nSteps;
};

struct GlobalKeys
{
  // The current parameterised potential parameters
  static ::sstbx::utility::Key< ::arma::vec>  POTENTIAL_PARAMS;
  static ::sstbx::utility::Key<ParamRange> POTENTIAL_SWEEP_RANGE;

};

}
}

#endif /* COMMON_DATA_H */
