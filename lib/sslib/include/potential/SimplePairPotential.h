/*
 * SimplePairPotential.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SIMPLE_PAIR_POTENTIAL_H
#define SIMPLE_PAIR_POTENTIAL_H

// DEFINES ////////////////////////////////////////////////
#define SPP_TYPE typename SimplePairPotential<FloatType>

// INCLUDES ///////////////////////////////////////////////

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>

#include <armadillo>

// Local includes
#include "common/AbstractFmidCell.h"
#include "common/Structure.h"
#include "common/Utils.h"
#include "potential/GenericPotentialEvaluator.h"
#include "potential/IParameterisable.h"
#include "potential/IPotential.h"
#include "potential/SimplePairPotentialData.h"


// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
namespace potential
{

template<typename FloatType = double>
class SimplePairPotential :
	public IPotential,
	public IParameterisable
{
public:

  /**
  /* A list from 0 to N of the species that each row (and column) of the parameter
  /* matrices corresponds to.  The entries should be unique.
  /*                      Na Cl
  /* epsilon_{ij} = Na (  1  0.5 )
  /*                CL ( 0.5  1  )
  /* speciesMap(0 => Na, 1 => Cl)
  /**/
  typedef typename SimplePairPotentialData<FloatType>::SpeciesList  SpeciesList;

  /**
  /* Combining rules for setting off-diagonal length/energy scale terms. See
  /* http://www.sklogwiki.org/SklogWiki/index.php/Combining_rules
  /* for good reference.
  /* If a rule is being used it will overwrite any off diagonal parameters.
  /**/
  enum CombiningRule
  {
    NONE,
    LORENTZ,
    BERTHELOT,
    LORENTZ_BERTHELOT,
    CUSTOM
  };

	typedef SimplePairPotentialData<FloatType>					      DataTyp;
	typedef typename arma::Mat<FloatType>						          Mat;
	typedef typename arma::Col<FloatType>::template fixed<3>	Vec3;

  static const unsigned int MAX_INTERACTION_VECTORS = 10000;

	SimplePairPotential(
		const size_t &				  numSpecies,
    const SpeciesList &     speciesList,
		const SPP_TYPE::Mat &		epsilon,
		const SPP_TYPE::Mat &		sigma,
		const FloatType &			  cutoffFactor,
		const SPP_TYPE::Mat &	  beta,
		const FloatType &			  m,
		const FloatType &			  n,
    const CombiningRule     combiningRule = NONE);

	virtual const ::std::string & getName() const;

	// From IParameterisable ////////////////////////////////////////

	virtual size_t getNumParams() const;
	virtual const ::std::string & getParamString() const;
	virtual ::arma::Col<FloatType> getParams() const;
	virtual void setParams(const ::arma::Col<FloatType> & params);
  virtual std::pair<arma::vec, bool>
    getParamsFromString(const std::string & str) const;

	// End IParameterisable //////////////////////////////////////////


  // From IPotential /////////////
  virtual ::boost::shared_ptr< IPotentialEvaluator > createEvaluator(const sstbx::common::Structure & structure) const;
  // End from IPotential /////////

  bool evaluate(SimplePairPotentialData<FloatType> & data) const;

private:

  typedef SimplePairPotentialData<FloatType> DataType;

  typedef GenericPotentialEvaluator< SimplePairPotential< FloatType >, SimplePairPotentialData< FloatType > >
    EvaluatorTyp;

	void initCutoff(const FloatType cutoff);

  void applyCombiningRule();

	void resetAccumulators(SimplePairPotentialData<FloatType> & data) const;

	const ::std::string		myName;

	mutable ::std::string	myParamString;

	/** Potential parameters */
	size_t					  myNumSpecies;
  SpeciesList       mySpeciesList;
	SPP_TYPE::Mat			myEpsilon;
	SPP_TYPE::Mat			mySigma;
	SPP_TYPE::Mat			myBeta;
  const double      myCutoffFactor;

	/** The powers of the sigma/r terms in the potential */
	FloatType				myN, myM;

  CombiningRule         myCombiningRule;

	arma::Mat<FloatType> 	rCutoff;
	arma::Mat<FloatType> 	rCutoffSq;
	arma::Mat<FloatType> 	eShift;
	arma::Mat<FloatType> 	fShift;
};

// IMPLEMENTATION /////////////////////////////////////

template <typename FloatType>
SimplePairPotential<FloatType>::SimplePairPotential(
	const size_t &				  numSpecies,
  const SpeciesList &     speciesList,
	const SPP_TYPE::Mat &		epsilon,
	const SPP_TYPE::Mat &		sigma,
	const FloatType &			  cutoffFactor,
	const SPP_TYPE::Mat &		beta,
	const FloatType &			  m,
	const FloatType &			  n,
  const CombiningRule     combiningRule):
	myName("Simple pair potential"),
	myNumSpecies(numSpecies),
  mySpeciesList(speciesList),
	myEpsilon(epsilon),
	mySigma(sigma),
	myBeta(beta),
	myM(m),
	myN(n),
  myCutoffFactor(cutoffFactor),
  myCombiningRule(combiningRule)
{
  applyCombiningRule();
	initCutoff(myCutoffFactor);
}


template<typename FloatType>
void SimplePairPotential<FloatType>::initCutoff(FloatType cutoff)
{
	// Initialise the cutoff matrices
	rCutoff.set_size(myNumSpecies, myNumSpecies);
	rCutoffSq.set_size(myNumSpecies, myNumSpecies);
	eShift.set_size(myNumSpecies, myNumSpecies);
	fShift.set_size(myNumSpecies, myNumSpecies);

	rCutoff		= cutoff * mySigma;
	rCutoffSq	= arma::pow(rCutoff, 2.0);

	// TODO: Change this to matrix notation rather than loops
	FloatType invRMaxN, invRMaxM;
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

template <typename FloatType>
void SimplePairPotential<FloatType>::applyCombiningRule()
{
  if(myCombiningRule == LORENTZ || myCombiningRule == LORENTZ_BERTHELOT)
  {
    // Apply the Lorenz combining rule
	  for(size_t i = 0; i < myNumSpecies - 1; ++i)
	  {
		  for(size_t j = i + 1; j < myNumSpecies; ++j)
		  {
			  mySigma(i, j) = mySigma(j, i) = 0.5 * (mySigma(i, i) + mySigma(j, j));
		  }
	  }
  }
  if(myCombiningRule == BERTHELOT || myCombiningRule == LORENTZ_BERTHELOT)
  {
    // Apply the Berthelot combining rule
	  for(size_t i = 0; i < myNumSpecies - 1; ++i)
	  {
		  for(size_t j = i + 1; j < myNumSpecies; ++j)
		  {
			  myEpsilon(i, j) = myEpsilon(j, i) = std::sqrt(myEpsilon(i, i) * myEpsilon(j, j));
		  }
	  }
  }
  if(myCombiningRule == CUSTOM)
  {
    double sum = 0.0;
    // Apply the Berthelot combining rule
	  for(size_t i = 0; i < myNumSpecies - 1; ++i)
	  {
		  for(size_t j = i + 1; j < myNumSpecies; ++j)
		  {
        sum = myEpsilon(i, i) + myEpsilon(j, j);
			  myEpsilon(i, j) = myEpsilon(j, i) = std::sqrt(16 - sum * sum);
		  }
	  }
  }
}

template <typename FloatType>
const ::std::string & SimplePairPotential<FloatType>::getName() const
{
	return myName;
}

template <typename FloatType>
size_t SimplePairPotential<FloatType>::getNumParams() const
{
  const float fNumSpecies = myNumSpecies;
  // 2 (eps/sig) * n * (n + 1) / 2 + n(species) = n( n + 3)
  return (size_t)(fNumSpecies * (fNumSpecies + 2.0));
}

template <typename FloatType>
::arma::Col<FloatType> SimplePairPotential<FloatType>::getParams() const
{
	::arma::Col<FloatType> params(getNumParams());

  size_t idx = 0;

	// Epsilon
	for(size_t i = 0; i < myNumSpecies; ++i)
	{
		for(size_t j = i; j < myNumSpecies; ++j, ++idx)
		{
			params(idx) = myEpsilon(i, j);
		}
	}
	// Sigma
	for(size_t i = 0; i < myNumSpecies; ++i)
	{
		for(size_t j = i; j < myNumSpecies; ++j, ++idx)
		{
			params(idx) = mySigma(i, j);
		}
	}
  for(size_t i = 0; i < myNumSpecies; ++i, ++idx)
  {
    params(idx) = mySpeciesList[i].ordinal();
  }

	return params;
}

template <typename FloatType>
void SimplePairPotential<FloatType>::setParams(const ::arma::Col<FloatType> & params)
{
	if(params.n_rows != getNumParams())
	{
		throw "setParams called with wrong number of parameters";
	}

  size_t idx = 0;

	// Epsilon
	for(size_t i = 0; i < myNumSpecies; ++i)
	{
		for(size_t j = i; j < myNumSpecies; ++j, ++idx)
		{
			myEpsilon(i, j) = params(idx);
		}
	}
  myEpsilon = arma::symmatu(myEpsilon);

	// Sigma
	for(size_t i = 0; i < myNumSpecies; ++i)
	{
		for(size_t j = i; j < myNumSpecies; ++j, ++idx)
		{
			mySigma(i, j) = params(idx);
		}
  }
  mySigma = arma::symmatu(mySigma);

  applyCombiningRule();

  // Initialise the cutoff matrices
  initCutoff(myCutoffFactor);

  // Set the species list
  for(size_t i = 0; i < myNumSpecies; ++i, ++idx)
  {
    mySpeciesList[i] = *::sstbx::common::AtomSpeciesId::values().operator[]((int)params[idx]);
  }

	// Reset the parameter string
	myParamString.clear();
}

template <typename FloatType>
std::pair<arma::vec, bool>
SimplePairPotential<FloatType>::getParamsFromString(const std::string & str) const
{
  using boost::trim;
  using boost::lexical_cast;
  using boost::bad_lexical_cast;
  using std::string;

  // Set up our tokenizer to split around space and tab
	typedef boost::tokenizer<boost::char_separator<char> > Tok;
	const boost::char_separator<char> sep(" \t");

  std::pair<arma::vec, bool> result;
  result.second = false;

  // Look for parameter indicators
  size_t nPos = str.find("n:");
  size_t ePos = str.find("e:");
  size_t sPos = str.find("s:");

  // Check to see if they were all gound
  if(nPos != string::npos && ePos != string::npos && sPos != string::npos)
  {
    // Try to get n
    bool foundN = false;
    unsigned int nSpecies = 0;
    try
    {
      string nStr = str.substr(nPos + 2, ePos - nPos - 2);
      trim(nStr);
      nSpecies = lexical_cast<unsigned int>(nStr);
      foundN = true;
    }
    catch(const bad_lexical_cast &)
    {
      foundN = true;
    }

    if(foundN)
    { 
      // Calculate the number of parameters for sigma/epsilon
      // WARNING: this may cause a problem because of the float/int conversion
      const unsigned int nTotal = nSpecies * (nSpecies + 1);
      const unsigned int nEach  = std::floor(nTotal * 0.5 + 0.5);
      result.first.set_size(nTotal);

      // Try to get epsilon
      const string eString = str.substr(ePos + 2, sPos - ePos - 2);
      Tok eToker(eString, sep);

      bool foundE = true;
      unsigned int i = 0;
      for(Tok::const_iterator eIt = eToker.begin();
        i < nEach && eIt != eToker.end();
        ++i, ++eIt)
      {
        try
        {
          result.first(i) = lexical_cast<double>(*eIt);
        }
        catch(const bad_lexical_cast &)
        {
          foundE = false;
          break;
        }
      }
      foundE &= i == nEach;

      if(foundE)
      { 
        // Try to get sigma
        const string sString = str.substr(sPos + 2);
        Tok sToker(sString, sep);

        bool foundS = true;
        unsigned int i = nEach;
        for(Tok::iterator sIt = sToker.begin();
          i < nTotal && sIt != sToker.end();
          ++i, ++sIt)
        {
          try
          {
            result.first(i) = lexical_cast<double>(*sIt);
          }
          catch(const bad_lexical_cast &)
          {
            foundS = false;
            break;
          }
        }
        foundS &= i == nTotal;
        
        if(foundS)
          result.second = true;

      } // end if(foundE)
    } // end if(foundN)
  } // end if n, s, e tokens found
  return result;
}

template <typename FloatType>
const ::std::string & SimplePairPotential<FloatType>::getParamString() const
{
	if(myParamString.empty())
	{
		::std::stringstream ss;
		ss << "n: " << myNumSpecies << " e:";
		// Epsilon
		for(size_t i = 0; i < myNumSpecies; ++i)
		{
			for(size_t j = i; j < myNumSpecies; ++j)
			{
				ss << " " << myEpsilon(i, j);
			}
		}
		ss << " s:";
		// Sigma
		for(size_t i = 0; i < myNumSpecies; ++i)
		{
			for(size_t j = i; j < myNumSpecies; ++j)
			{
				ss << " " << mySigma(i, j);
			}
		}
		myParamString = ss.str();
	}
	return myParamString;
}

template<typename FloatType>
bool SimplePairPotential<FloatType>::evaluate(
	SimplePairPotentialData<FloatType> & data) const
{
	using namespace arma;
	using ::std::vector;

	FloatType rSq;
	FloatType sigmaOModR, invRM, invRN;
	FloatType dE, modR, modF;
  size_t speciesI, speciesJ;  // Species indices
	SPP_TYPE::Vec3 r, f;        // Displacement and force vectors
  SPP_TYPE::Vec3 posI, posJ;  // Position vectors

	resetAccumulators(data);

	vector<SPP_TYPE::Vec3> imageVectors;

	const sstbx::common::AbstractFmidCell<FloatType> & uc = data.unitCell;
	
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
			uc.getAllVectorsWithinCutoff(posI, posJ, rCutoff(speciesI, speciesJ), imageVectors, MAX_INTERACTION_VECTORS);

      // Check that there aren't too many interaction vectors
      if(imageVectors.size() > MAX_INTERACTION_VECTORS)
      {
        return false;
      }

      BOOST_FOREACH(r, imageVectors)
			{			
				// Get the distance squared
				rSq = dot(r, r);

				// Check that distance is less than cut-off
				if(rSq < rCutoffSq(speciesI, speciesJ) && rSq > 0.0)
				{
					modR = sqrt(rSq);

					sigmaOModR = mySigma(speciesI, speciesJ) / modR;

					invRM = pow(sigmaOModR, myM);
					invRN = pow(sigmaOModR, myN) * myBeta(speciesI, speciesJ);

					// Calculate the energy delta
					dE = 2.0 * myEpsilon(speciesI, speciesJ) * (invRM - invRN) -
						eShift(speciesI, speciesJ) + (modR - rCutoff(speciesI, speciesJ)) * fShift(speciesI, speciesJ);

					// Magnitude of the force
					modF = 2.0 *  myEpsilon(speciesI, speciesJ) *
						(myM * invRM - myN * invRN) / modR - fShift(speciesI, speciesJ);
					f = modF / modR * r;

					// Make sure we get energy/force correct for self-interaction
					if(i != j)
					{
						f *= 2.0;
						dE *= 2.0;
					}

					// Update system values
					// energy
					data.totalEnthalpy += dE;
					// force
					data.forces.col(i) -= f;
					if(i != j)
						data.forces.col(j) += f;
					
					// stress, diagonal is element wise multiplication of force and position
					// vector components
					data.stressMtx.diag() += f % r;
					
					data.stressMtx(1, 2) += 0.5 * (f(1)*r(2)+f(2)*r(1));
					data.stressMtx(2, 0) += 0.5 * (f(2)*r(0)+f(0)*r(2));
					data.stressMtx(0, 1) += 0.5 * (f(0)*r(1)+f(1)*r(0));

				}
			}
		}
	}

	// Symmetrise stress matrix
	data.stressMtx(2, 1) = data.stressMtx(1, 2);
	data.stressMtx(0, 2) = data.stressMtx(2, 0);
	data.stressMtx(1, 0) = data.stressMtx(0, 1);

	// And convert to absoloute values
	const FloatType invVolume = 1.0 / uc.getVolume();
	data.stressMtx *= invVolume;

  // Completed successfully
  return true;
}

template <typename FloatType>
::boost::shared_ptr< IPotentialEvaluator >
  SimplePairPotential<FloatType>::createEvaluator(const sstbx::common::Structure & structure) const
{
  // Build the data from the structure
  ::boost::shared_ptr< SimplePairPotentialData< FloatType > > data(
    new SimplePairPotentialData<FloatType>(structure, mySpeciesList)
  );

  // Create the evaluator
  return ::boost::shared_ptr<IPotentialEvaluator>(new EvaluatorTyp(*this, data));
}


template <typename FloatType>
void SimplePairPotential<FloatType>::resetAccumulators(SimplePairPotentialData<FloatType> & data) const
{
	data.totalEnthalpy = 0.0;
	data.forces.fill(0.0);
	data.stressMtx.fill(0.0);
}

}}

#endif /* SIMPLE_PAIR_POTENTIAL_H */

