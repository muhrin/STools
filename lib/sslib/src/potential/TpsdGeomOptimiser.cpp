/*
 * TpsdGeomOptimiser.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/TpsdGeomOptimiser.h"

#include "common/UnitCell.h"

// NAMESPACES ////////////////////////////////


namespace sstbx {
namespace potential {


// CONSTANTS ////////////////////////////////////////////////


const size_t TpsdGeomOptimiser::DEFAULT_MAX_STEPS = 50000;
const double TpsdGeomOptimiser::DEFAULT_TOLERANCE = 1e-13;
const double TpsdGeomOptimiser::DEFAULT_MIN_NORM_VOLUME = 0.05;

// IMPLEMENTATION //////////////////////////////////////////////////////////


TpsdGeomOptimiser::TpsdGeomOptimiser(
	const IPotential & potential,
	const size_t maxSteps,
	const double tolerance,
  const double minNormVolume):
myPotential(potential),
myMaxSteps(maxSteps),
myTolerance(tolerance),
myMinNormVolume(minNormVolume)
{}

const IPotential * TpsdGeomOptimiser::getPotential() const
{
	return &myPotential;
}

bool TpsdGeomOptimiser::optimise(
	::sstbx::common::Structure & structure,
  const common::OptionalConstMat33 externalPressure) const
{
  ::boost::shared_ptr<IPotentialEvaluator> evaluator = myPotential.createEvaluator(structure);

  common::UnitCell * const unitCell = structure.getUnitCell();

  if(unitCell)
  {
	  return optimise(
      structure,
      *unitCell,
      *evaluator,
      DEFAULT_MAX_STEPS,
      DEFAULT_TOLERANCE,
      externalPressure
    );
  }
  else
  {
	  return optimise(
      structure,
      *evaluator,
      DEFAULT_MAX_STEPS,
      DEFAULT_TOLERANCE,
      externalPressure
    );
  }
}


bool TpsdGeomOptimiser::optimise(
	::sstbx::common::Structure & structure,
	PotentialData & data,
  const common::OptionalConstMat33 externalPressure) const
{
  ::boost::shared_ptr<IPotentialEvaluator> evaluator = myPotential.createEvaluator(structure);

  common::UnitCell * const unitCell = structure.getUnitCell();

  bool outcome;
  if(unitCell)
  {
	  outcome = optimise(
      structure,
      *unitCell,
      *evaluator,
      DEFAULT_MAX_STEPS,
      DEFAULT_TOLERANCE,
      externalPressure
    );
  }
  else
  {
	  outcome = optimise(
      structure,
      *evaluator,
      DEFAULT_MAX_STEPS,
      DEFAULT_TOLERANCE,
      externalPressure
    );
  }

  // Copy over data from the geometry optimisation
  data = evaluator->getData();

	return outcome;
}

bool TpsdGeomOptimiser::optimise(
  common::Structure &   structure,
  IPotentialEvaluator & evaluator,
	const size_t maxSteps,
	const double eTol,
  const common::OptionalConstMat33 externalPressure) const
{
  // Set up the external pressure
  ::arma::mat33 pressureMtx;
  pressureMtx.fill(0.0);
  if(externalPressure)
  {
    pressureMtx = *externalPressure;
  }
  const double pressureMean = ::arma::trace(pressureMtx) / 3.0;

  // Get data about the structure to be optimised
  PotentialData & data = evaluator.getData();

	double h, h0, dH;

	// Position matrices, current are in data.myPos
  ::arma::mat deltaPos(3, data.numParticles);
	// Forces, current are in data.myForces
  ::arma::mat f0(3, data.numParticles), deltaF(3, data.numParticles);

	double xg, gg;
  ::arma::vec3 f;

	data.forces.fill(1.0);
	deltaPos.fill(0.0);

	// Initialisation of variables
	dH	= std::numeric_limits<double>::max();
	h	= 1.0;

	const size_t numIonsSq	= data.numParticles * data.numParticles;

	bool converged = false;
  size_t numLastEvaluationsWithProblem = 0;

	// Set the initial step size so get mooving
	double step = eTol * 1e8;
	for(size_t i = 0; !converged && i < maxSteps; ++i)
	{
		h0 = h;
		f0 = data.forces;

		// Evaluate the potential
		if(!evaluator.evalPotential().second)
    {
      // Couldn't evaluate potential for some reason.  Probably the unit cell
      // has collapsed and there are too many r12 vectors to evaluate.
      ++numLastEvaluationsWithProblem;
    }
    else
    {
      // That evaluation was fine, so reset counter
      numLastEvaluationsWithProblem = 0;
    }

		// Now balance forces
		// (do sum of values for each component and divide by number of particles)
		f = sum(data.forces, 1) / data.numParticles;

		data.forces.row(0) += f(0);
		data.forces.row(1) += f(1);
		data.forces.row(2) += f(2);

		h = data.internalEnergy;

		deltaF	= data.forces - f0;
		// The accu function will do the sum of all elements
		// and the % operator does the Shure product i.e.
		// element wise multiplication of two matrices
		xg		= accu(deltaPos % deltaF);
		gg		= accu(deltaF % deltaF);


		if(fabs(xg) > 0.0)
			step = fabs(xg / gg);

		// Move the particles on by a step, saving the old positions
		deltaPos		= step * data.forces;
		data.pos		+= deltaPos;

    // Tell the structure about the new positions
    structure.setAtomPositions(data.pos);

		dH = h - h0;

		converged = fabs(dH) < eTol;
	}

  // Only a successful optimisation if it has converged
  // and the last potential evaluation had no problems
  return converged && numLastEvaluationsWithProblem == 0;
}

bool TpsdGeomOptimiser::optimise(
  common::Structure &   structure,
  common::UnitCell &    unitCell,
  IPotentialEvaluator & evaluator,
	const size_t maxSteps,
	const double eTol,
  const common::OptionalConstMat33 externalPressure) const
{
  // Set up the external pressure
  ::arma::mat33 pressureMtx;
  pressureMtx.fill(0.0);
  if(externalPressure)
  {
    pressureMtx = *externalPressure;
  }
  const double pressureMean = ::arma::trace(pressureMtx) / 3.0;

  // Get data about the structure to be optimised
  PotentialData & data = evaluator.getData();

	double h, h0, dH;

	// Stress matrices
  ::arma::mat33	s, s0, deltaS, deltaLatticeCar;
	// Position matrices, current are in data.myPos
  ::arma::mat deltaPos(3, data.numParticles);
	// Forces, current are in data.myForces
  ::arma::mat f0(3, data.numParticles), deltaF(3, data.numParticles);

  ::arma::mat33 latticeCar;
	double gamma, volume, volumeSq, gammaNumIonsOVolume;
	double xg, gg;
  ::arma::vec3 f;

	data.forces.fill(1.0);
	deltaPos.fill(0.0);
	deltaLatticeCar.fill(0.0);
	latticeCar = unitCell.getOrthoMtx();

	// Initialisation of variables
	dH	= std::numeric_limits<double>::max();
	h	= 1.0;
	s.fill(1.0);

	const size_t numIonsSq	= data.numParticles * data.numParticles;

	bool converged = false;
  size_t numLastEvaluationsWithProblem = 0;
	// Set the initial step size so get mooving
	double step = eTol * 1e8;
	for(size_t i = 0; !converged && i < maxSteps; ++i)
	{
		h0 = h;
		f0 = data.forces;
		s0 = s;

		volume		= unitCell.getVolume();
		volumeSq	= volume * volume;

		// Evaluate the potential
		if(!evaluator.evalPotential().second)
    {
      // Couldn't evaluate potential for some reason.  Probably the unit cell
      // has collapsed and there are too many r12 vectors to evaluate.
      ++numLastEvaluationsWithProblem;
    }
    else
    {
      // That evaluation was fine, so reset counter
      numLastEvaluationsWithProblem = 0;
    }

		// Now balance forces
		// (do sum of values for each component and divide by number of particles)
		f = sum(data.forces, 1) / data.numParticles;

		data.forces.row(0) += f(0);
		data.forces.row(1) += f(1);
		data.forces.row(2) += f(2);

		// TODO: Check this gamma = 0 as it seems a little odd
		gamma = 0.0;
		data.internalEnergy += gamma * numIonsSq / volume;

		gammaNumIonsOVolume = gamma * numIonsSq / volumeSq;
		data.stressMtx.diag() += gammaNumIonsOVolume;

		s = data.stressMtx * latticeCar;

		h = data.internalEnergy + pressureMean * volume;

		deltaF	= data.forces - f0;
		// The accu function will do the sum of all elements
		// and the % operator does the Shure product i.e.
		// element wise multiplication of two matrices
		xg		= accu(deltaPos % deltaF);
		gg		= accu(deltaF % deltaF);

		deltaS	= s - s0;
		xg		+= accu(deltaLatticeCar % deltaS);
		gg		+= accu(deltaS % deltaS);


		if(fabs(xg) > 0.0)
			step = fabs(xg / gg);

		// Move the particles on by a step, saving the old positions
		deltaPos		= step * data.forces;
		data.pos		+= deltaPos;

		// Fractionalise coordinates and wrap coordinates
    unitCell.cartsToFracInplace(data.pos);
    unitCell.wrapVecsFracInplace(data.pos);

		// Move on cell vectors to relax strain
		deltaLatticeCar = step * (s - pressureMtx * latticeCar);
		latticeCar += deltaLatticeCar;
		unitCell.setOrthoMtx(latticeCar);

		// Finally re-orthogonalise the ion positions
    unitCell.fracsToCartInplace(data.pos);

    // Tell the structure about the new positions
    structure.setAtomPositions(data.pos);

		dH = h - h0;

		converged = fabs(dH) < eTol;

		if((i % 40 == 0) && unitCell.getNormVolume() < myMinNormVolume)
    {
      // Cell has collapsed
      converged = false;
      break;
    }
	}

	// Wrap the particle positions so they stay in the central unit cell
	unitCell.wrapVecsInplace(data.pos);

	return converged && numLastEvaluationsWithProblem == 0;
}


}
}

