/*
 * IControllableOptimiser.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_CONTROLLABLE_OPTIMISER_H
#define I_CONTROLLABLE_OPTIMISER_H

// INCLUDES /////////////////////////////////////////////
#include "potential/IGeomOptimiser.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {

// FORWARD DECLARATIONS ////////////////////////////////////

namespace potential {
class IOptimisationController;

class IControllableOptimiser : public IGeomOptimiser
{
public:
  virtual IOptimisationController * getController() = 0;
  virtual void setController(IOptimisationController & controller) = 0;
};


}
}

#endif /* I_CONTROLLABLE_OPTIMISER_H */
