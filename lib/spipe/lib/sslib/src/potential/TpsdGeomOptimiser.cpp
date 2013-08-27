/*
 * TpsdGeomOptimiser.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/TpsdGeomOptimiser.h"

#include <sstream>

#include "SSLib.h"
#include "common/UnitCell.h"
#include "potential/OptimisationSettings.h"

#define TPSD_GEOM_OPTIMISER_DEBUG (SSLIB_DEBUG & 0)

#if TPSD_GEOM_OPTIMISER_DEBUG
#  include <sstream>
#  include "common/AtomSpeciesDatabase.h"
#  include "io/ResReaderWriter.h"
#endif

// NAMESPACES ////////////////////////////////
namespace sstbx {
namespace potential {

#if TPSD_GEOM_OPTIMISER_DEBUG
class TpsdGeomOptimiserDebugger
{
public:
  const static unsigned int SAVE_EVERY_N_STEPS = 1000;
  TpsdGeomOptimiserDebugger()
  {}

  void postOptStepDebugHook(common::Structure & structure, const unsigned int step) const
  {
    if(step % SAVE_EVERY_N_STEPS == 0)
    {
      ::std::stringstream ss;
      ss << structure.getName() << "-" << step << "-" << "debug.res";
      myWriter.writeStructure(structure, ss.str(), mySpeciesDb);
    }
  }

private:
  const common::AtomSpeciesDatabase mySpeciesDb;
  io::ResReaderWriter myWriter;
};
#endif

// CONSTANTS ////////////////////////////////////////////////

const unsigned int TpsdGeomOptimiser::DEFAULT_MAX_STEPS = 50000;
const double TpsdGeomOptimiser::DEFAULT_ENERGY_TOLERANCE = 1e-12;
const double TpsdGeomOptimiser::DEFAULT_FORCE_TOLERANCE = 1e-10;
const unsigned int TpsdGeomOptimiser::CHECK_CELL_EVERY_N_STEPS = 20;
const double TpsdGeomOptimiser::CELL_MIN_NORM_VOLUME = 0.02;
const double TpsdGeomOptimiser::CELL_MAX_ANGLE_SUM = 360.0;
const double TpsdGeomOptimiser::MAX_STEPSIZE = 10.0;
static const double INITIAL_STEPSIZE = 0.02;

// IMPLEMENTATION //////////////////////////////////////////////////////////

TpsdGeomOptimiser::TpsdGeomOptimiser(PotentialPtr potential) :
    myPotential(potential), myEnergyTolerance(DEFAULT_ENERGY_TOLERANCE), myMaxSteps(DEFAULT_MAX_STEPS),
    myForceTolerance(DEFAULT_FORCE_TOLERANCE)
{
}

double
TpsdGeomOptimiser::getEnergyTolerance() const
{
  return myEnergyTolerance;
}

void
TpsdGeomOptimiser::setEnergyTolerance(const double tolerance)
{
  myEnergyTolerance = tolerance;
}

double TpsdGeomOptimiser::getForceTolerance() const
{
  return myForceTolerance;
}

void TpsdGeomOptimiser::setForceTolerance(const double tolerance)
{
  myForceTolerance = tolerance;
}

unsigned int
TpsdGeomOptimiser::getMaxSteps() const
{
  return myMaxSteps;
}

void
TpsdGeomOptimiser::setMaxSteps(const unsigned int maxSteps)
{
  myMaxSteps = maxSteps;
}

IPotential *
TpsdGeomOptimiser::getPotential()
{
  return myPotential.get();
}

const IPotential *
TpsdGeomOptimiser::getPotential() const
{
  return myPotential.get();
}

OptimisationOutcome
TpsdGeomOptimiser::optimise(::sstbx::common::Structure & structure,
    const OptimisationSettings & options) const
{
  OptimisationData optData;
  return optimise(structure, optData, options);
}

OptimisationOutcome
TpsdGeomOptimiser::optimise(::sstbx::common::Structure & structure, OptimisationData & data,
    const OptimisationSettings & options) const
{
  ::boost::shared_ptr< IPotentialEvaluator> evaluator = myPotential->createEvaluator(structure);

  common::UnitCell * const unitCell = structure.getUnitCell();
  OptimisationSettings localSettings = options;

  if(!localSettings.maxSteps)
    localSettings.maxSteps.reset(myMaxSteps);
  if(!localSettings.pressure)
    localSettings.pressure.reset(::arma::zeros< ::arma::mat>(3, 3));
  if(!localSettings.optimisationType)
    localSettings.optimisationType.reset(OptimisationSettings::Optimise::ATOMS_AND_LATTICE);

  OptimisationOutcome outcome;
  if(unitCell)
    outcome = optimise(structure, *unitCell, data, *evaluator, localSettings);
  else
    outcome = optimise(structure, data, *evaluator, localSettings);

  data.saveToStructure(structure);

  return outcome;
}

OptimisationOutcome
TpsdGeomOptimiser::optimise(common::Structure & structure, OptimisationData & optimisationData,
    IPotentialEvaluator & evaluator, const OptimisationSettings & settings) const
{
  SSLIB_ASSERT(settings.maxSteps.is_initialized());

  // Get data about the structure to be optimised
  PotentialData & data = evaluator.getData();

  double h, h0, dH;

  // Position matrices, current are in data.myPos
  ::arma::mat deltaPos(3, data.numParticles);
  // Forces, current are in data.myForces
  ::arma::mat f0(3, data.numParticles), deltaF(3, data.numParticles);
  ::arma::rowvec fSqNorm(::arma::zeros(1, data.numParticles));

  double xg, gg;
  ::arma::vec3 f;

  data.forces.ones();
  deltaPos.zeros();

  // Initialisation of variables
  dH = std::numeric_limits< double>::max();
  h = 1.0;

  const double dNumAtoms = static_cast<double>(data.numParticles);

  bool converged = false;
  size_t numLastEvaluationsWithProblem = 0;

  // Set the initial step size so get moving
  double step = INITIAL_STEPSIZE;
  for(size_t i = 0; !converged && i < *settings.maxSteps; ++i)
  {
    // Save the energy and forces from last time around
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

    h = data.internalEnergy;

    deltaF = data.forces - f0;

    fSqNorm = ::arma::sum(data.forces % data.forces);
    // The % operator does the Schur product i.e.
    // element wise multiplication of two matrices
    // The accu function will do the sum of all elements
    xg = accu(deltaPos % deltaF);
    gg = accu(deltaF % deltaF);

    if(fabs(xg) > 0.0)
      step = fabs(xg / gg);

    // Move the particles on by a step, saving the old positions
    deltaPos = step * data.forces;
    data.pos += deltaPos;

    // Tell the structure about the new positions
    structure.setAtomPositions(data.pos);

    dH = h - h0;
    converged = hasConverged(dH / dNumAtoms, fSqNorm.max(), settings);
  }

  // Only a successful optimisation if it has converged
  // and the last potential evaluation had no problems
  if(numLastEvaluationsWithProblem != 0)
    return OptimisationOutcome::failure(OptimisationError::ERROR_EVALUATING_POTENTIAL,
        "Potential evaluation errors during optimisation");
  if(!converged)
  {
    ::std::stringstream ss;
    ss << "Failed to converge after " << *settings.maxSteps << "steps";
    return OptimisationOutcome::failure(OptimisationError::FAILED_TO_CONVERGE, ss.str());
  }

  populateOptimistaionData(optimisationData, structure, data);
  return OptimisationOutcome::success();
}

OptimisationOutcome
TpsdGeomOptimiser::optimise(common::Structure & structure, common::UnitCell & unitCell,
    OptimisationData & optimisationData, IPotentialEvaluator & evaluator,
    const OptimisationSettings & settings) const
{
  SSLIB_ASSERT(settings.maxSteps.is_initialized());
  SSLIB_ASSERT(settings.pressure.is_initialized());

#if TPSD_GEOM_OPTIMISER_DEBUG
  TpsdGeomOptimiserDebugger debugger;
#endif

  // Set up the external pressure
  ::arma::mat33 pressureMtx = *settings.pressure;
  const double pressureMean = ::arma::trace(pressureMtx) / 3.0;

  // Get data about the structure to be optimised
  PotentialData & data = evaluator.getData();

  // Stress matrices
  ::arma::mat33 s, s0, deltaS, deltaLatticeCar;
  // Position matrices, current are in data.myPos
  ::arma::mat deltaPos(3, data.numParticles);
  // Forces, current are in data.myForces
  ::arma::mat f0(3, data.numParticles), deltaF(3, data.numParticles);
  ::arma::rowvec fSqNorm(::arma::zeros(1, data.numParticles));

  ::arma::mat33 latticeCar;
  double gamma, volume, volumeSq;
  double xg, gg;

  data.forces.ones();
  deltaPos.zeros();
  deltaLatticeCar.zeros();
  latticeCar = unitCell.getOrthoMtx();

  // Initialisation of variables
  double dH = std::numeric_limits< double>::max(); // Change in enthalpy between steps
  double h = 1.0; // Enthalpy = U + pV
  double h0;
  s.ones();

  const double dNumAtoms = static_cast<double>(data.numParticles);

  bool converged = false;
  size_t numLastEvaluationsWithProblem = 0;

  // Set the initial step size so get moving
  double step = INITIAL_STEPSIZE;
  for(unsigned int i = 0; !converged && i < *settings.maxSteps; ++i)
  {
    h0 = h;
    f0 = data.forces;
    s0 = s;

    volume = unitCell.getVolume();
    volumeSq = volume * volume;

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

    // Calculate the strain matrix
    s = data.stressMtx * latticeCar;
    // Calculate the enthalpy
    h = data.internalEnergy + pressureMean * volume;

    deltaF = data.forces - f0;

    xg = gg = 0.0;
    if(*settings.optimisationType & OptimisationSettings::Optimise::ATOMS)
    {
      fSqNorm = ::arma::sum(data.forces % data.forces);
      // The accu function will do the sum of all elements
      // and the % operator does the Shure product i.e.
      // element wise multiplication of two matrices
      xg = accu(deltaPos % deltaF);
      gg = accu(deltaF % deltaF);
    }

    deltaS = s - s0;
    if(*settings.optimisationType & OptimisationSettings::Optimise::LATTICE)
    {
      xg += accu(deltaLatticeCar % deltaS);
      gg += accu(deltaS % deltaS);
    }

    if(fabs(xg) > 0.0)
      step = ::std::fabs(xg / gg);

    if(*settings.optimisationType & OptimisationSettings::Optimise::ATOMS)
    {
      // Move the particles on by a step, saving the old positions
      deltaPos = step * data.forces;
      data.pos += deltaPos;
    }

    // Fractionalise coordinates and wrap coordinates
    unitCell.cartsToFracInplace(data.pos);
    unitCell.wrapVecsFracInplace(data.pos);

    if(*settings.optimisationType & OptimisationSettings::Optimise::LATTICE)
    {
      // Move on cell vectors to relax strain
      deltaLatticeCar = step * (s - pressureMtx * latticeCar);
      settings.applyLatticeConstraints(structure, latticeCar, deltaLatticeCar);
      latticeCar += deltaLatticeCar;

      if(!unitCell.setOrthoMtx(latticeCar))
      {
        // The unit cell matrix has become singular
        converged = false;
        break;
      }
    }

    // Finally re-orthogonalise the ion positions
    unitCell.fracsToCartInplace(data.pos);

    // Tell the structure about the new positions
    structure.setAtomPositions(data.pos);

    dH = h - h0;

    converged = hasConverged(dH / dNumAtoms, fSqNorm.max(), settings);

#if TPSD_GEOM_OPTIMISER_DEBUG
    debugger.postOptStepDebugHook(structure, i);
#endif

    if((i + 1 % CHECK_CELL_EVERY_N_STEPS == 0) && !cellReasonable(unitCell))
    {
      return OptimisationOutcome::failure(OptimisationError::PROBLEM_WITH_STRUCTURE,
          "Unit cell has collapsed.");
    }
  }

  // Wrap the particle positions so they stay in the central unit cell
  unitCell.wrapVecsInplace(data.pos);

  // Only a successful optimisation if it has converged
  // and the last potential evaluation had no problems
  if(numLastEvaluationsWithProblem != 0)
    return OptimisationOutcome::failure(OptimisationError::ERROR_EVALUATING_POTENTIAL,
        "Potential evaluation errors during optimisation");
  if(!converged)
  {
    ::std::stringstream ss;
    ss << "Failed to converge after " << *settings.maxSteps << "steps";
    return OptimisationOutcome::failure(OptimisationError::FAILED_TO_CONVERGE, ss.str());
  }

  populateOptimistaionData(optimisationData, structure, data);
  return OptimisationOutcome::success();
}

bool
TpsdGeomOptimiser::cellReasonable(const sstbx::common::UnitCell & unitCell) const
{
  // Do a few checks to see if the cell has collapsed
  if(unitCell.getNormVolume() < CELL_MIN_NORM_VOLUME)
    return false;

  const double (&params)[6] = unitCell.getLatticeParams();

  if(params[3] + params[4] + params[5] > CELL_MAX_ANGLE_SUM)
    return false;

  return true;
}

void
TpsdGeomOptimiser::populateOptimistaionData(OptimisationData & optData,
    const common::Structure & structure, const PotentialData & potData) const
{
  const common::UnitCell * const unitCell = structure.getUnitCell();

  optData.internalEnergy.reset(potData.internalEnergy);
  const double pressure = -::arma::trace(potData.stressMtx) / 3.0;
  optData.pressure.reset(pressure);
  if(unitCell)
  {
    optData.enthalpy.reset(potData.internalEnergy + *optData.pressure * unitCell->getVolume());
  }
  optData.ionicForces.reset(potData.forces);
  optData.stressMtx.reset(potData.stressMtx);
}

bool TpsdGeomOptimiser::hasConverged(const double deltaEnergyPerIon, const double maxForceSq,
    const OptimisationSettings & options) const
{
  bool converged = true;

  // TODO: Add lattice stress
  //if(*options.optimisationType & OptimisationSettings::Optimise::LATTICE)

  if(*options.optimisationType & OptimisationSettings::Optimise::ATOMS)
    converged &= deltaEnergyPerIon < myEnergyTolerance && maxForceSq < myForceTolerance * myForceTolerance;

  return converged;
}

}
}

