/*
 * IOptimisationController.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_OPTIMISATION_CONTROLLER_H
#define I_OPTIMISATION_CONTROLLER_H

// INCLUDES /////////////////////////////////////////////
#include "potential/IGeomOptimiser.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {
namespace common {
class Structure;
}
// FORWARD DECLARATIONS ////////////////////////////////////

namespace potential {
struct OptimisationData;

class IOptimisationController
{
public:
  virtual bool optimisationStarting(common::Structure & structure) = 0;
  virtual bool stepFinished(
    const int step,
    common::Structure & structure,
    const OptimisationData & optimisationData
  ) = 0;
  virtual void optimisationFinished(
    const OptimisationOutcome & outcome,
    common::Structure & structure,
    const OptimisationData & optimisationData
  ) = 0;
};


}
}

#endif /* I_OPTIMISATION_CONTROLLER_H */
