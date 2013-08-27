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

#include <string>
#include <vector>

#include <armadillo>

#include <spl/utility/HeterogeneousMap.h>

// FORWARD DECLARATIONS ////////////////////////////////////


namespace spipe {
namespace common {

class ParamRange
{
public:
  ParamRange() {}
  ParamRange(const size_t dims);

  bool fromStrings(const ::std::vector< ::std::string> & paramStrings);

  ::std::vector<double> from;
  ::std::vector<double> step;
  ::std::vector<int> nSteps;
private:

  bool parseParamString(const size_t idx, const ::std::string & paramString);
};

struct GlobalKeys
{
  // The current parameterised potential parameters
  static ::spl::utility::Key< ::std::vector<double> >  POTENTIAL_PARAMS;
  static ::spl::utility::Key<ParamRange> POTENTIAL_SWEEP_RANGE;

};

}
}

#endif /* COMMON_DATA_H */
