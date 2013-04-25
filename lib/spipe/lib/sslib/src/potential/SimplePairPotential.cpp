/*
 * SimplePairPotentialData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/SimplePairPotential.h"

#include <memory>

#include "common/DistanceCalculator.h"
#include "common/UnitCell.h"
#include "utility/IndexingEnums.h"

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace potential {

// Using 0.5 prefactor as 2^(1/6) s is the equilibrium separation of the centres.
// i.e. the diameter
const double SimplePairPotential::RADIUS_FACTOR = 0.5 * ::std::pow(2, 1.0/6.0);
const double SimplePairPotential::MIN_SEPARATION_SQ = 1e-20;

unsigned int SimplePairPotential::numParams(const unsigned int numSpecies)
{
  return 0;
}


SimplePairPotential::SimplePairPotential(
  common::AtomSpeciesDatabase & atomSpeciesDb,
  const SpeciesList &           speciesList,
	const ::arma::mat &		        epsilon,
	const ::arma::mat &		        sigma,
	const double 			            cutoffFactor,
	const ::arma::mat &		        beta,
	const double 			            n,
	const double    			        m,
  const CombiningRule::Value    combiningRule):
	myName("Simple pair potential"),
  myAtomSpeciesDb(atomSpeciesDb),
	myNumSpecies(speciesList.size()),
  mySpeciesList(speciesList),
	myEpsilon(epsilon),
	mySigma(sigma),
	myBeta(beta),
	myM(m),
	myN(n),
  myCutoffFactor(cutoffFactor),
  myCombiningRule(combiningRule)
{
  SSLIB_ASSERT(myNumSpecies == myEpsilon.n_rows);
  SSLIB_ASSERT(myEpsilon.is_square());
  SSLIB_ASSERT(myNumSpecies == mySigma.n_rows);
  SSLIB_ASSERT(mySigma.is_square());
  SSLIB_ASSERT(myNumSpecies == myBeta.n_rows);
  SSLIB_ASSERT(myBeta.is_square());

  applyCombiningRule();
	initCutoff(myCutoffFactor);
  updateSpeciesDb();
}


void SimplePairPotential::initCutoff(double cutoff)
{
	// Initialise the cutoff matrices
	rCutoff.set_size(myNumSpecies, myNumSpecies);
	rCutoffSq.set_size(myNumSpecies, myNumSpecies);
	eShift.set_size(myNumSpecies, myNumSpecies);
	fShift.set_size(myNumSpecies, myNumSpecies);

	rCutoff		= cutoff * mySigma;
	rCutoffSq	= arma::pow(rCutoff, 2.0);

	double invRMaxN, invRMaxM;
	for(size_t i = 0; i < myNumSpecies; ++i)
	{
		for(size_t j = 0; j < myNumSpecies; ++j)
		{
			invRMaxN = pow(mySigma(i, j) / rCutoff(i, j), myN) * myBeta(i, j);

			invRMaxM = pow(mySigma(i, j) / rCutoff(i, j), myM);

			eShift(i, j) = 2.0 * myEpsilon(i, j) * (invRMaxM - invRMaxN);

			fShift(i, j) = 2.0 * myEpsilon(i, j) * (myM * invRMaxM / rCutoff(i, j) -
				myN * invRMaxN / rCutoff(i, j));
		}
	}
}


void SimplePairPotential::applyCombiningRule()
{
  applyEnergyRule(myEpsilon, myCombiningRule);
  applySizeRule(mySigma, myCombiningRule);
}


const ::std::string & SimplePairPotential::getName() const
{
	return myName;
}


size_t SimplePairPotential::getNumParams() const
{
  // epsilon: n(n + 1) / 2
  // sigma: n(n + 1) / 2
  // beta: n(n + 1) / 2
  // m-n: 2
  return 3 * myNumSpecies * (myNumSpecies + 1) / 2 + 2;
}


IParameterisable::PotentialParams
SimplePairPotential::getParams() const
{
	IParameterisable::PotentialParams params(getNumParams());

  size_t idx = 0;

	// Epsilon
	for(size_t i = 0; i < myNumSpecies; ++i)
  {
		for(size_t j = i; j < myNumSpecies; ++j)
			params[idx++] = myEpsilon(i, j);
	}
	// Sigma
	for(size_t i = 0; i < myNumSpecies; ++i)
	{
		for(size_t j = i; j < myNumSpecies; ++j)
			params[idx++] = mySigma(i, j);
	}
  // Beta
  for(size_t i = 0; i < myNumSpecies; ++i)
  {
    for(size_t j = i; j < myNumSpecies; ++j)
			params[idx++] = myBeta(i, j);
  }
  // N-M
  params[idx++] = myN;
  params[idx++] = myM;

	return params;
}


void SimplePairPotential::setParams(const IParameterisable::PotentialParams & params)
{
  SSLIB_ASSERT_MSG(params.size() == getNumParams(), "Called setParams with wrong number of parameters");

  size_t idx = 0;

	// Epsilon
	for(size_t i = 0; i < myNumSpecies; ++i)
	{
		for(size_t j = i; j < myNumSpecies; ++j, ++idx)
			myEpsilon(i, j) = params[idx];
	}
  myEpsilon = arma::symmatu(myEpsilon);

	// Sigma
	for(size_t i = 0; i < myNumSpecies; ++i)
  {
		for(size_t j = i; j < myNumSpecies; ++j, ++idx)
			mySigma(i, j) = params[idx];
  }
  mySigma = arma::symmatu(mySigma);

	// Beta
	for(size_t i = 0; i < myNumSpecies; ++i)
  {
		for(size_t j = i; j < myNumSpecies; ++j, ++idx)
			myBeta(i, j) = params[idx];
  }
  myBeta = arma::symmatu(myBeta);

  myN = params[idx++];
  myM = params[idx++];

  applyCombiningRule();

  // Initialise the cutoff matrices
  initCutoff(myCutoffFactor);

	// Reset the parameter string
	myParamString.clear();
  // Update the species database
  updateSpeciesDb();
}

bool SimplePairPotential::evaluate(const common::Structure & structure, SimplePairPotentialData & data) const
{
  using namespace utility::cart_coords_enum;
	using ::std::vector;

	double rSq;
	double sigmaOModR, invRM, invRN;
	double dE, modR, modF;
  size_t speciesI, speciesJ;  // Species indices
	::arma::vec3 r, f;        // Displacement and force vectors
  ::arma::vec3 posI, posJ;  // Position vectors

	resetAccumulators(data);

	vector< ::arma::vec3> imageVectors;

  const common::DistanceCalculator & distCalc = structure.getDistanceCalculator();
	
  bool problemDuringCalculation = false;

	// Loop over all particle pairs (including self-interaction)
	for(size_t i = 0; i < data.numParticles; ++i)
	{
		speciesI = data.species[i];
    if(speciesI == DataType::IGNORE_ATOM)
      continue;

		posI = data.pos.col(i);

		for(size_t j = i; j < data.numParticles; ++j)
		{
			speciesJ = data.species[j];
      if(speciesJ == DataType::IGNORE_ATOM)
        continue;

			posJ = data.pos.col(j);

      // TODO: Buffer rSqs as getAllVectorsWithinCutoff needs to calculate it anyway!
			imageVectors.clear();
      if(!distCalc.getVecsBetween(posI, posJ, rCutoff(speciesI, speciesJ), imageVectors, MAX_INTERACTION_VECTORS, MAX_CELL_MULTIPLES))
      {
        // We reached the maximum number of interaction vectors so indicate that there was a problem
        problemDuringCalculation = true;
      }

      BOOST_FOREACH(r, imageVectors)
			{			
				// Get the distance squared
				rSq = dot(r, r);

				// Check that distance isn't near the 0 as this will cause near-singular values
				if(rSq > MIN_SEPARATION_SQ)
				{
					modR = sqrt(rSq);

					sigmaOModR = mySigma(speciesI, speciesJ) / modR;

					invRN = pow(sigmaOModR, myN);
					invRM = pow(sigmaOModR, myM) * myBeta(speciesI, speciesJ);

					// Calculate the energy delta
					dE = 4.0 * myEpsilon(speciesI, speciesJ) * (invRN - invRM) -
						eShift(speciesI, speciesJ) + (modR - rCutoff(speciesI, speciesJ)) * fShift(speciesI, speciesJ);

					// Magnitude of the force
					modF = 4.0 *  myEpsilon(speciesI, speciesJ) *
						(myN * invRN - myM * invRM) / modR - fShift(speciesI, speciesJ);
					f = modF / modR * r;

					// Make sure we get energy/force correct for self-interaction
					if(i == j)
					{
						f *= 0.5;
						dE *= 0.5;
					}

					// Update system values
					// energy
					data.internalEnergy += dE;
					// force
					data.forces.col(i) -= f;
					if(i != j)
						data.forces.col(j) += f;
					
					// stress, diagonal is element wise multiplication of force and position
					// vector components
					data.stressMtx.diag() += f % r;

					data.stressMtx(Y, Z) += 0.5 * (f(Y)*r(Z)+f(Z)*r(Y));
					data.stressMtx(Z, X) += 0.5 * (f(Z)*r(X)+f(X)*r(Z));
					data.stressMtx(X, Y) += 0.5 * (f(X)*r(Y)+f(Y)*r(X));
				}
			}
		}
	}

	// Symmetrise stress matrix
	data.stressMtx(2, 1) = data.stressMtx(1, 2);
	data.stressMtx(0, 2) = data.stressMtx(2, 0);
	data.stressMtx(1, 0) = data.stressMtx(0, 1);

	// Now balance forces
	// (do sum of values for each component and divide by number of particles)
  f = sum(data.forces, 1) / static_cast<double>(data.numParticles);
	data.forces.row(X) -= f(Y);
	data.forces.row(Y) -= f(X);
	data.forces.row(Z) -= f(Z);

  // Convert stress matrix to absolute values
  const common::UnitCell * const unitCell = structure.getUnitCell();
  if(unitCell)
  {
	  const double invVolume = 1.0 / unitCell->getVolume();
	  data.stressMtx *= invVolume;
  }

  // Completed successfully
  return !problemDuringCalculation;
}


::boost::optional<double>
SimplePairPotential::getPotentialRadius(const ::sstbx::common::AtomSpeciesId::Value id) const
{
  ::boost::optional<double> radius;
  for(size_t i = 0; i < mySpeciesList.size(); ++i)
  {
    if(mySpeciesList[i] == id)
    {
      radius.reset(RADIUS_FACTOR * mySigma(i, i));
      break;
    }
  }
  return radius;
}


::boost::shared_ptr<IPotentialEvaluator>
SimplePairPotential::createEvaluator(const sstbx::common::Structure & structure) const
{
  // Build the data from the structure
  ::std::auto_ptr<SimplePairPotentialData> data(new SimplePairPotentialData(structure, mySpeciesList));

  // Create the evaluator
  return ::boost::shared_ptr<IPotentialEvaluator>(new Evaluator(*this, structure, data));
}

IParameterisable * SimplePairPotential::getParameterisable()
{
  return this;
}


void SimplePairPotential::resetAccumulators(SimplePairPotentialData & data) const
{
	data.internalEnergy = 0.0;
	data.forces.zeros();
	data.stressMtx.zeros();
}

void SimplePairPotential::updateSpeciesDb()
{
  BOOST_FOREACH(common::AtomSpeciesId::Value species, mySpeciesList)
  {
    myAtomSpeciesDb.setRadius(species, *getPotentialRadius(species));
  }
}

}
}
