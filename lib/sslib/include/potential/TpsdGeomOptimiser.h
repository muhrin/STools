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

// DEFINES //////////////////////////////////////////////
#define TPSD_GEOM_OPTIMISER_TPARAMS	<typename FloatType>
#define TPSD_GEOM_OPTIMISER_TTYPE	TpsdGeomOptimiser<FloatType>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
namespace potential
{

template <typename FloatType = double>
class TpsdGeomOptimiser : public IGeomOptimiser
{
public:
	TpsdGeomOptimiser(
		const IPotential & potential,
		const size_t maxSteps = DEFAULT_MAX_STEPS,
		const FloatType tolerance = DEFAULT_TOLERANCE,
    const FloatType minNormVolume = DEFAULT_MIN_NORM_VOLUME);

	// IGeomOptimiser interface //////////////////////////////
  virtual const IPotential * getPotential() const;

	virtual bool optimise(
    ::sstbx::common::Structure &  structure,
    const ::arma::mat * const     externalPressure = NULL) const;

	virtual bool optimise(
		::sstbx::common::Structure &  structure,
    ::boost::shared_ptr< StandardData<FloatType> > & data,
    const ::arma::mat * const     externalPressure = NULL) const;

	// End IGeomOptimiser interface

	bool optimise(
    IPotentialEvaluator &     evaluator,
		const size_t              maxSteps,
		const FloatType           eTol,
    const ::arma::mat * const externalPressure) const;

	static const size_t		DEFAULT_MAX_STEPS;
	static const FloatType	DEFAULT_TOLERANCE;
	static const FloatType	DEFAULT_CELL_ANGLE_THRESHOLD;
  static const FloatType  DEFAULT_MIN_NORM_VOLUME;
	static const FloatType	MAX_DH_TO_H_RATIO;

private:

	const IPotential & myPotential;

	const size_t		myMaxSteps;
	const FloatType		myTolerance;
  const FloatType   myMinNormVolume;
};

// CONSTANTS ////////////////////////////////////////////////

template TPSD_GEOM_OPTIMISER_TPARAMS
const size_t TPSD_GEOM_OPTIMISER_TTYPE::DEFAULT_MAX_STEPS = 50000;

template TPSD_GEOM_OPTIMISER_TPARAMS
const FloatType TPSD_GEOM_OPTIMISER_TTYPE::DEFAULT_TOLERANCE = 1e-13;

template TPSD_GEOM_OPTIMISER_TPARAMS
const FloatType TPSD_GEOM_OPTIMISER_TTYPE::DEFAULT_MIN_NORM_VOLUME = 0.05;

template TPSD_GEOM_OPTIMISER_TPARAMS
const FloatType TPSD_GEOM_OPTIMISER_TTYPE::MAX_DH_TO_H_RATIO = 10000;

template TPSD_GEOM_OPTIMISER_TPARAMS
const FloatType TPSD_GEOM_OPTIMISER_TTYPE::DEFAULT_CELL_ANGLE_THRESHOLD = 15;

// IMPLEMENTATION //////////////////////////////////////////////////////////

template TPSD_GEOM_OPTIMISER_TPARAMS
TPSD_GEOM_OPTIMISER_TTYPE::TpsdGeomOptimiser(
	const IPotential & potential,
	const size_t maxSteps,
	const FloatType tolerance,
  const FloatType minNormVolume):
myPotential(potential),
myMaxSteps(maxSteps),
myTolerance(tolerance),
myMinNormVolume(minNormVolume)
{}


template TPSD_GEOM_OPTIMISER_TPARAMS
const IPotential * TPSD_GEOM_OPTIMISER_TTYPE::getPotential() const
{
	return &myPotential;
}

template TPSD_GEOM_OPTIMISER_TPARAMS
bool TPSD_GEOM_OPTIMISER_TTYPE::optimise(
	::sstbx::common::Structure &  structure,
  const ::arma::mat * const     externalPressure) const
{
  ::boost::shared_ptr<IPotentialEvaluator> evaluator = myPotential.createEvaluator(structure);

	const bool outcome = optimise(
    *evaluator,
    DEFAULT_MAX_STEPS,
    DEFAULT_TOLERANCE,
    externalPressure
  );

  ::boost::shared_ptr<StandardData<FloatType> > data = evaluator->getData();

	// Copy the new positions back to the structure
	structure.setAtomPositionsDescendent(data->pos);

  // Copy over the unit cell
  *structure.getUnitCell() = data->unitCell;

	return outcome;
}

template TPSD_GEOM_OPTIMISER_TPARAMS
bool TPSD_GEOM_OPTIMISER_TTYPE::optimise(
	::sstbx::common::Structure &                    structure,
	::boost::shared_ptr<StandardData<FloatType> > & data,
  const ::arma::mat * const                       externalPressure) const
{
  ::boost::shared_ptr< IPotentialEvaluator > evaluator = myPotential.createEvaluator(structure);

	const bool outcome = optimise(
    *evaluator,
    DEFAULT_MAX_STEPS,
    DEFAULT_TOLERANCE,
    externalPressure
  );

  data = evaluator->getData();

	// Copy the new positions back to the structure
	structure.setAtomPositionsDescendent(data->pos);

  // Copy over the unit cell
  *structure.getUnitCell() = data->unitCell;

	return outcome;
}

template TPSD_GEOM_OPTIMISER_TPARAMS
bool TPSD_GEOM_OPTIMISER_TTYPE::optimise(
  IPotentialEvaluator &     evaluator,
	const size_t              maxSteps,
	const FloatType           eTol,
  const ::arma::mat * const externalPressure) const
{
	typedef typename arma::Col<FloatType>::template fixed<3>	Vec3;
	typedef typename arma::Mat<FloatType>						Mat3;
	typedef typename arma::Mat<FloatType>::template fixed<3, 3>	Mat33;

  // Set up the external pressure
  Mat33 pressure;
  pressure.fill(0.0);
  if(externalPressure)
  {
    pressure = *externalPressure;
  }
  const double pressureMean = arma::trace(pressure) / 3.0;

  // Get data about the structure to be optimised
  StandardData<FloatType> & data = *evaluator.getData();

	FloatType h, h0, dH;

	// Stress matrices
	Mat33	s, s0, deltaS, deltaLatticeCar;
	// Position matrices, current are in data.myPos
	Mat3 deltaPos(3, data.numParticles);
	// Forces, current are in data.myForces
	Mat3	f0(3, data.numParticles), deltaF(3, data.numParticles);

	sstbx::common::AbstractFmidCell & uc = data.unitCell;

	Mat33 latticeCar;
	FloatType gamma, volume, volumeSq, gammaNumIonsOVolume;
	FloatType xg, gg;
	Vec3 f;

	data.forces.fill(1.0);
	deltaPos.fill(0.0);
	deltaLatticeCar.fill(0.0);
	latticeCar			= uc.getOrthoMtx();


	// Initialisation of variables
	dH	= std::numeric_limits<FloatType>::max();
	h	= 1.0;
	s.fill(1.0);

	const size_t numIonsSq	= data.numParticles * data.numParticles;

	bool converged = false;
	// Set the initial step size so get mooving
	FloatType step = 0.001;
	for(size_t i = 0; !converged && i < maxSteps; ++i)
	{
		h0 = h;
		f0 = data.forces;
		s0 = s;

		volume		= uc.getVolume();
		volumeSq	= volume * volume;

		// Evaluate the potential
		if(!evaluator.evalPotential())
    {
      // Couldn't evaluate potential for some reason.  Probably the unit cell
      // has collapsed and there are too many r12 vectors to evaluate.  Bail.
      break;
    }

		// Now balance forces
		// (do sum of values for each component and divide by number of particles)
		f = sum(data.forces, 1) / data.numParticles;

		data.forces.row(0) += f(0);
		data.forces.row(1) += f(1);
		data.forces.row(2) += f(2);

		// TODO: Check this gamma = 0 as it seems a little odd
		gamma = 0.0;
		data.totalEnthalpy += gamma * numIonsSq / volume;

		gammaNumIonsOVolume = gamma * numIonsSq / volumeSq;
		data.stressMtx.diag() += gammaNumIonsOVolume;

		s = data.stressMtx * latticeCar;

		h = data.totalEnthalpy + pressureMean * volume;

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

		// Fractionalise coordinates
		const Mat33 & fracMtx = uc.getFracMtx();
		data.pos		= fracMtx * data.pos;
		data.pos		-= arma::floor(data.pos);

		// Move on cell vectors to relax strain
		deltaLatticeCar = step * (s - pressure * latticeCar);
		latticeCar		+= deltaLatticeCar;
		uc.setOrthoMtx(latticeCar);

		// Finally re-orthogonalise the ion positions
		data.pos		= latticeCar * data.pos;

		dH = h - h0;

		converged = fabs(dH) < eTol;


		// TODO: Temporary escape condition to catch pathological cases!
		if(dH / h > MAX_DH_TO_H_RATIO)
    {
      converged = false;
			break;
    }

		if((i % 40 == 0) && data.unitCell.getNormVolume() < myMinNormVolume)
    {
      // Cell has collapsed
      converged = false;
      break;
    }
	}

	// Wrap the particle positions so they stay in the central unit cell
	uc.wrapVecsInplace(data.pos);

	return converged;

}

}}

#endif /* TPSD_GEOM_OPTIMISER_H */
