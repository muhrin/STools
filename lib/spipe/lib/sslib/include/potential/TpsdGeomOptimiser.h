/*
 * TpsdGeomOptimiser.h
 *
 * Two-point Step Size Gradient Methods - Barzilai and Borwein
 * IMA Journal of Numerical Analysis (1988) 8, 141-148
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef TPSD_GEOM_OPTIMISER_H
#define TPSD_GEOM_OPTIMISER_H

// INCLUDES /////////////////////////////////////////////
#include <limits>

#include <armadillo>

#include "potential/IGeomOptimiser.h"
#include "potential/IPotential.h"
#include "potential/IPotentialEvaluator.h"

#include "common/Structure.h"
#include "common/Types.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {

// FORWARD DECLARATIONS ////////////////////////////////////
namespace common {
class UnitCell;
}
namespace potential {

class TpsdGeomOptimiser : public IGeomOptimiser
{
public:

  typedef ::sstbx::UniquePtr<IPotential>::Type PotentialPtr;

	static const unsigned int DEFAULT_MAX_STEPS;
	static const double	DEFAULT_TOLERANCE;

	TpsdGeomOptimiser(PotentialPtr potential);

  double getTolerance() const;
  void setTolerance(const double tolerance);

  unsigned int getMaxSteps() const;
  void setMaxSteps(const unsigned int maxSteps);

	// IGeomOptimiser interface //////////////////////////////
  virtual IPotential * getPotential();
  virtual const IPotential * getPotential() const;

	virtual OptimisationOutcome optimise(
    ::sstbx::common::Structure & structure,
    const OptimisationSettings & options
  ) const;
	virtual OptimisationOutcome optimise(
		::sstbx::common::Structure &  structure,
    OptimisationData & data,
    const OptimisationSettings & options
  ) const;

	// End IGeomOptimiser interface

	OptimisationOutcome optimise(
    common::Structure & structure,
    OptimisationData & optimistaionData,
    IPotentialEvaluator & evaluator,
		const double eTol,
    const OptimisationSettings & options
  ) const;

	OptimisationOutcome optimise(
    common::Structure & structure,
    common::UnitCell & unitCell,
    OptimisationData & optimistaionData,
    IPotentialEvaluator & evaluator,
		const double eTol,
    const OptimisationSettings & options
  ) const;

private:

  static const unsigned int CHECK_CELL_EVERY_N_STEPS;
  static const double CELL_MIN_NORM_VOLUME;
  static const double CELL_MAX_ANGLE_SUM;
  static const double MAX_STEPSIZE;

  bool cellReasonable(const common::UnitCell & unitCell) const;
  void populateOptimistaionData(
    OptimisationData & optData,
    const common::Structure & structure,
    const PotentialData & potData
  ) const;

	PotentialPtr myPotential;

	double myTolerance;
  unsigned int myMaxSteps;
};

}
}

#endif /* TPSD_GEOM_OPTIMISER_H */
