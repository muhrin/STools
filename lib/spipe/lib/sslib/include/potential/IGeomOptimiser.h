/*
 * IGeomOptimiser.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_GEOM_OPTIMISER_H
#define I_GEOM_OPTIMISER_H

// INCLUDES /////////////////////////////////////////////

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <armadillo>

#include "common/Types.h"
#include "utility/Outcome.h"

// DEFINES //////////////////////////////////////////////


namespace sstbx {

// FORWARD DECLARATIONS ////////////////////////////////////
namespace common {
class Structure;
}
namespace potential {
struct PotentialData;
class IPotential;
struct OptimisationSettings;

struct OptimisationError
{
  enum Value
  {
    FAILED_TO_CONVERGE,
    ERROR_EVALUATING_POTENTIAL,
    INTERNAL_ERROR
  };
};

typedef utility::OutcomeWithErrorCode<OptimisationError::Value> OptimisationOutcome;

class IGeomOptimiser
{
public:

	virtual ~IGeomOptimiser() {}

  /**
  /* Get the potential being used by the geometry optimiser.  Not all
  /* geometry optimisers need to have a potential in which case NULL
  /* will be returned.
  /**/
  virtual IPotential * getPotential() = 0;
  virtual const IPotential * getPotential() const = 0;

	virtual OptimisationOutcome optimise(
    common::Structure & structure,
    const OptimisationSettings & options
  ) const = 0;
	virtual OptimisationOutcome optimise(
		common::Structure & structure,
    PotentialData & data,
    const OptimisationSettings & options
  ) const = 0;
};

}
}

#endif /* I_GEOM_OPTIMISER_H */
