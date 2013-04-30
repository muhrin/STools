/*
 * LandscapeExplorerOptimiser.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef LANDSCAPE_EXPLORER_OPTIMISER_H
#define LANDSCAPE_EXPLORER_OPTIMISER_H

// INCLUDES /////////////////////////////////////////////
#include "SSLib.h"

#include "potential/IGeomOptimiser.h"
#include "potential/LandscapeExplorer.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {

// FORWARD DECLARATIONS ////////////////////////////////////
namespace utility {
class IStructureComparator;
}
namespace potential {
class IControllableOptimiser;

class LandscapeExplorerOptimiser : public IGeomOptimiser
{
public:
  typedef UniquePtr<IControllableOptimiser>::Type OptimiserPtr;
  typedef UniquePtr<utility::IStructureComparator>::Type ComparatorPtr;

  LandscapeExplorerOptimiser(OptimiserPtr optimiser, ComparatorPtr comparator);

  // From IGeomOptimiser ///////
  virtual IPotential * getPotential();
  virtual const IPotential * getPotential() const;

	virtual OptimisationOutcome optimise(
    common::Structure & structure,
    const OptimisationSettings & options
  ) const;
	virtual OptimisationOutcome optimise(
		common::Structure & structure,
    OptimisationData & data,
    const OptimisationSettings & options
  ) const;
  // End from IGeomOptimiser ///

private:
  OptimiserPtr myOptimiser;
  LandscapeExplorer myExplorer;
};


}
}

#endif /* LANDSCAPE_EXPLORER_OPTIMISER_H */
