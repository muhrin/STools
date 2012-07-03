/*
 * AbstractFmidCell.h
 *
 * Abstract Fast Minimum Image Cell
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef ABSTRACT_FMID_CELL_H
#define ABSTRACT_FMID_CELL_H

// INCLUDES /////////////////////////////////////////////
#include "SSLib.h"

#include <cmath>
#include <vector>
//#include <math.h>

#include <armadillo>

// Local includes
#include "common/Constants.h"
#include "utility/StableComparison.h"

// DEFINES ////////////////////////////////////////////////
#define ABSTRACT_FMID_CELL AbstractFmidCell<FloatType>
#define ABSTRACT_FMID_CELL_TYPE typename AbstractFmidCell<FloatType>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx { namespace common {

template<typename FloatType = double>
class AbstractFmidCell
{
public:

	// TYPEDEFS ////////////////////////
	// Need to use the typename keyword to make it clear that these are types
	typedef typename arma::Mat<FloatType>				Mat;
	typedef typename arma::Mat<FloatType>::template fixed<3, 3>	Mat33;
	typedef typename arma::Col<FloatType>::template fixed<3>		Vec3;

  static const unsigned int MAX_OUT_VECTORS = 100000;

	// Copy constructor
	AbstractFmidCell(const AbstractFmidCell<FloatType> & toCopy);

	AbstractFmidCell(
		const FloatType a, const FloatType b, const FloatType c,
		const FloatType alpha, const FloatType beta, const FloatType gamma);

	explicit AbstractFmidCell(const FloatType (&latticeParams)[6]);

	/**
	/* Get the minimum distance between two cartesian points respecting the boundary
	/* conditions
	/**/
	virtual FloatType getDistanceMinimumImg(
		const typename AbstractFmidCell<FloatType>::Vec3 & a,
		const typename AbstractFmidCell<FloatType>::Vec3 & b) const;

	virtual FloatType getDistanceSqMinimumImg(
		const typename AbstractFmidCell<FloatType>::Vec3 & a,
		const typename AbstractFmidCell<FloatType>::Vec3 & b) const;

	const FloatType (&getLatticeParams() const)[6];

	virtual typename AbstractFmidCell<FloatType>::Vec3 getVecMinimumImg(
		const typename AbstractFmidCell<FloatType>::Vec3 & a,
		const typename AbstractFmidCell<FloatType>::Vec3 & b) const;

	virtual void getAllVectorsWithinCutoff(
		const typename AbstractFmidCell<FloatType>::Vec3 & cart1,
		const typename AbstractFmidCell<FloatType>::Vec3 & cart2,
		const FloatType cutoff,
		std::vector<typename AbstractFmidCell<FloatType>::Vec3> & outVectors,
    const size_t    maxVectors = MAX_OUT_VECTORS) const;

	virtual void getAllDistancesWithinCutoff(
		const typename AbstractFmidCell<FloatType>::Vec3 & cart1,
		const typename AbstractFmidCell<FloatType>::Vec3 & cart2,
		const FloatType cutoff,
		std::vector<FloatType> & outVectors,
		const size_t maxCellMultiples = 0) const;

	FloatType getNumPlaneRepetitionsToBoundSphere(
		const typename AbstractFmidCell<FloatType>::Vec3 & boundDir,
		const typename AbstractFmidCell<FloatType>::Vec3 & planeVecA,
		const typename AbstractFmidCell<FloatType>::Vec3 & planeVecB,
		const FloatType radius) const;

	typename AbstractFmidCell<FloatType>::Vec3 wrapVec(const typename AbstractFmidCell<FloatType>::Vec3 & toWrap) const;
	typename AbstractFmidCell<FloatType>::Vec3 & wrapVecInplace(typename AbstractFmidCell<FloatType>::Vec3 & toWrap) const;

	typename AbstractFmidCell<FloatType>::Mat & wrapVecsInplace(typename AbstractFmidCell<FloatType>::Mat & carts) const;
	typename AbstractFmidCell<FloatType>::Mat & wrapVecsInplaceFrac(typename AbstractFmidCell<FloatType>::Mat & fracs) const;

	typename AbstractFmidCell<FloatType>::Vec3 wrapVecFrac(const typename AbstractFmidCell<FloatType>::Vec3 & toWrap) const;
	typename AbstractFmidCell<FloatType>::Vec3 & wrapVecFracInplace(typename AbstractFmidCell<FloatType>::Vec3 & toWrap) const;

	virtual Vec3 randomPoint() const;

	/////////////////////////////////////////////////////////
	// VECTOR TRANSFORMATION METHODS ////////////////////////

	virtual Vec3 fractionalise(const typename AbstractFmidCell<FloatType>::Vec3 & cartesian) const;

	virtual Vec3 & fractionaliseInplace(typename AbstractFmidCell<FloatType>::Vec3 & cart) const;

	/**
	/* Fractionalise multiple cartesian vectors.  This method takes an argument that
	/* must be a 3 x n matrix.
	/**/
	virtual Mat & fractionaliseInplace(typename AbstractFmidCell<FloatType>::Mat & carts) const;

	virtual Vec3 orthogonalise(const typename AbstractFmidCell<FloatType>::Vec3 & frac) const;

	virtual Vec3 & orthogonaliseInplace(typename AbstractFmidCell<FloatType>::Vec3 & frac) const;

	virtual Mat & orthogonaliseInplace(typename AbstractFmidCell<FloatType>::Mat & fracs) const;

	// END ORTHOGONALISATION METHODS //////////////////////////////

	virtual const Mat33 & getOrthoMtx() const;

	virtual void setOrthoMtx(const typename AbstractFmidCell<FloatType>::Mat33 & orthoMtx);
	const Mat33 getFracMtx() const;

	// TODO: RENAME THESE!
	FloatType getLongestVector() const;
	FloatType getLongestVectorSq() const;

	virtual FloatType getVolume() const;
  
  FloatType getNormVolume() const;

	FloatType setVolume(const FloatType volume);


	//typename AbstractFmidCell<FloatType>::Mat33 niggliReduce();
  bool niggliReduce();

protected:

	/** Initialise the unit cell from lattice parameters */
	void init(
		const FloatType a, const FloatType b, const FloatType c,
		const FloatType alpha, const FloatType beta, const FloatType gamma);
	/** Initialise the unit cell from an orthogonalisation matrix */
	void init(const Mat33 & orthoMtx);
	void initOrthoAndFracMatrices();
	void initLatticeParams();
	void initRest();

	/** The unit cell matrix where columns represent basis vectors */
	typename AbstractFmidCell<FloatType>::Mat33 myOrthoMtx;

	/** The inverse of the orthogonalisation matrix */
	typename AbstractFmidCell<FloatType>::Mat33 myFracMtx;

	FloatType	myLatticeParams[6];

	FloatType	myVolume;
};

// IMPLEMENTATION /////////////////////////////////////////////

template <typename FloatType>
AbstractFmidCell<FloatType>::AbstractFmidCell(const AbstractFmidCell<FloatType> & toCopy)
{
	init(toCopy.getOrthoMtx());
}


template <typename FloatType>
AbstractFmidCell<FloatType>::AbstractFmidCell(
	const FloatType a, const FloatType b, const FloatType c,
	const FloatType alpha, const FloatType beta, const FloatType gamma)
{
	init(a, b, c, alpha, beta, gamma);
}

template <typename FloatType>
AbstractFmidCell<FloatType>::AbstractFmidCell(const FloatType (&latticeParams)[6])
{
	// TODO: Tidy this up a bit
	init(latticeParams[0], latticeParams[1], latticeParams[2],
		latticeParams[3], latticeParams[4], latticeParams[5]);
}

template <typename FloatType>
FloatType AbstractFmidCell<FloatType>::getDistanceMinimumImg(
	const ABSTRACT_FMID_CELL_TYPE::Vec3 & a, const ABSTRACT_FMID_CELL_TYPE::Vec3 & b) const
{
	return sqrt(getDistanceSqMinimumImg(a, b));
}

template <typename FloatType>
FloatType AbstractFmidCell<FloatType>::getDistanceSqMinimumImg(
	const typename AbstractFmidCell<FloatType>::Vec3 & a,
	const typename AbstractFmidCell<FloatType>::Vec3 & b) const
{
	using namespace arma;
	typename AbstractFmidCell<FloatType>::Vec3 dr = getVecMinimumImg(a, b);
	return dot(dr, dr);
}

template <typename FloatType>
const FloatType (&AbstractFmidCell<FloatType>::getLatticeParams() const)[6]
{
	return myLatticeParams;
}

template <typename FloatType>
typename AbstractFmidCell<FloatType>::Vec3 AbstractFmidCell<FloatType>::getVecMinimumImg(
	const typename AbstractFmidCell<FloatType>::Vec3 & a,
	const typename AbstractFmidCell<FloatType>::Vec3 & b) const
{
	using namespace arma;

	// Make sure cart1 and 2 are in the unit cell at the origin
	const typename AbstractFmidCell<FloatType>::Vec3 		aUnit	= wrapVec(a);
	const typename AbstractFmidCell<FloatType>::Vec3 		bUnit	= wrapVec(b);
	const typename AbstractFmidCell<FloatType>::Vec3		dR		= bUnit - aUnit;
	FloatType		minModDRSq= dot(dR, dR);
	const FloatType minModDR= sqrt(minModDRSq);

	// Get the lattice vectors
	const typename AbstractFmidCell<FloatType>::Vec3 A = myOrthoMtx.col(0);
	const typename AbstractFmidCell<FloatType>::Vec3 B = myOrthoMtx.col(1);
	const typename AbstractFmidCell<FloatType>::Vec3 C = myOrthoMtx.col(2);

	// Maximum multiple of cell vectors we need to go to
	int maxA = (int)ceil(getNumPlaneRepetitionsToBoundSphere(A, B, C, minModDR));
	int maxB = (int)ceil(getNumPlaneRepetitionsToBoundSphere(B, A, C, minModDR));
	int maxC = (int)ceil(getNumPlaneRepetitionsToBoundSphere(C, A, B, minModDR));

	typename AbstractFmidCell<FloatType>::Vec3 dFrac, dRImg, minDR = dR;
	FloatType modDRSq;
	for(int a = -maxA; a <= maxA; ++a)
	{
		for(int b = -maxB; b <= maxB; ++b)
		{
			for(int c = -maxC; c <= maxC; ++c)
			{
				// TODO: Replace this with matrix multiplication
				dFrac[0] = a;
				dFrac[1] = b;
				dFrac[2] = c;

        dRImg = myOrthoMtx * dFrac + dR;
				
				modDRSq = dot(dRImg, dRImg);
				if(modDRSq < minModDRSq)
				{
					minModDRSq	= modDRSq;
					minDR		= dRImg;
				}
			}
		}
	}

	return minDR;
}

template <typename FloatType>
void AbstractFmidCell<FloatType>::getAllVectorsWithinCutoff(
	const typename AbstractFmidCell<FloatType>::Vec3 & cart1,
	const typename AbstractFmidCell<FloatType>::Vec3 & cart2,
	const FloatType cutoff,
	std::vector<typename AbstractFmidCell<FloatType>::Vec3> & outVectors,
  const size_t maxVectors) const
{
	using namespace arma;

	// Make sure cart1 and 2 are in the unit cell at the origin
	const typename AbstractFmidCell<FloatType>::Vec3 cart1Unit = wrapVec(cart1);
	const typename AbstractFmidCell<FloatType>::Vec3 cart2Unit = wrapVec(cart2);

	// Get the lattice vectors
	const typename AbstractFmidCell<FloatType>::Vec3 A = myOrthoMtx.col(0);
	const typename AbstractFmidCell<FloatType>::Vec3 B = myOrthoMtx.col(1);
	const typename AbstractFmidCell<FloatType>::Vec3 C = myOrthoMtx.col(2);

	// Maximum multiple of cell vectors we need to go to
	int maxA = (int)ceil(getNumPlaneRepetitionsToBoundSphere(A, B, C, cutoff));
	int maxB = (int)ceil(getNumPlaneRepetitionsToBoundSphere(B, A, C, cutoff));
	int maxC = (int)ceil(getNumPlaneRepetitionsToBoundSphere(C, A, B, cutoff));

	const typename AbstractFmidCell<FloatType>::Vec3 dR = cart2Unit - cart1Unit;
	const FloatType	cutoffSq	= cutoff * cutoff;

	typename AbstractFmidCell<FloatType>::Vec3 dFrac, dRImg;
  size_t numVectors = 0;
	for(int a = -maxA; a <= maxA; ++a)
	{
		for(int b = -maxB; b <= maxB; ++b)
		{
			for(int c = -maxC; c <= maxC; ++c)
			{
				// TODO: Replace this with matrix multiplication
				dFrac[0] = a;
				dFrac[1] = b;
				dFrac[2] = c;

				dRImg = myOrthoMtx * dFrac + dR;

				if(dot(dRImg, dRImg) < cutoffSq)
				{
					outVectors.push_back(dRImg);
          ++numVectors;
				}
			}
      if(numVectors > maxVectors)
      {
#ifdef SSLIB_DEBUG
        ::std::cerr << "Maximum vectors reached" << std::endl;
#endif
        return;
      }
		}
	}
}

/*
template <typename FloatType>
FloatType AbstractFmidCell<FloatType>::getNumPlaneRepetitionsToBoundSphere(
	const typename arma::Row<FloatType>::fixed<3> & boundDir,
	const typename arma::Row<FloatType>::fixed<3> & planeVecA,
	const typename arma::Row<FloatType>::fixed<3> & planeVecB,
	const FloatType radius) const
{
	// The vector normal to the plane
	const typename AbstractFmidCell<FloatType>::Vec3 normal = arma::cross(planeVecA, planeVecB);

	return radius / getVolume() * std::sqrt(arma::dot(normal, normal));
}
*/

template <typename FloatType>
FloatType AbstractFmidCell<FloatType>::getNumPlaneRepetitionsToBoundSphere(
	const typename AbstractFmidCell<FloatType>::Vec3 & boundDir,
	const typename AbstractFmidCell<FloatType>::Vec3 & planeVecA,
	const typename AbstractFmidCell<FloatType>::Vec3 & planeVecB,
	const FloatType radius) const
{
	// The vector normal to the plane
	const typename AbstractFmidCell<FloatType>::Vec3 normal = arma::cross(planeVecA, planeVecB);

	return radius / getVolume() * std::sqrt(arma::dot(normal, normal));
}

template <typename FloatType>
typename AbstractFmidCell<FloatType>::Vec3 AbstractFmidCell<FloatType>::randomPoint() const
{
	// Generate a random point in fractional coords and then
	// orthogonalise it
	typename AbstractFmidCell<FloatType>::Vec3 x;
	x.randu();
	return orthogonaliseInplace(x);
}

template <typename FloatType>
inline typename AbstractFmidCell<FloatType>::Vec3
AbstractFmidCell<FloatType>::fractionalise(const typename AbstractFmidCell<FloatType>::Vec3 & cartesian) const
{
	return myFracMtx * cartesian;
}

template <typename FloatType>
inline typename AbstractFmidCell<FloatType>::Vec3
AbstractFmidCell<FloatType>::orthogonalise(const typename AbstractFmidCell<FloatType>::Vec3 & frac) const
{
	return myOrthoMtx * frac;
}

template <typename FloatType>
const typename AbstractFmidCell<FloatType>::Mat33 & AbstractFmidCell<FloatType>::getOrthoMtx() const
{
	return myOrthoMtx;
}

template <typename FloatType>
void AbstractFmidCell<FloatType>::setOrthoMtx(const typename AbstractFmidCell<FloatType>::Mat33 & orthoMtx)
{
	myOrthoMtx = orthoMtx;
	init(orthoMtx);
}

template <typename FloatType>
const typename AbstractFmidCell<FloatType>::Mat33 AbstractFmidCell<FloatType>::getFracMtx() const
{
	return myFracMtx;
}

template <typename FloatType>
typename AbstractFmidCell<FloatType>::Vec3 AbstractFmidCell<FloatType>::wrapVec(const typename AbstractFmidCell<FloatType>::Vec3 & toWrap) const
{
	using namespace arma;

	// Fractionalise it
	typename AbstractFmidCell<FloatType>::Vec3 frac = myFracMtx * toWrap;

	// Wrap it
	wrapVecFracInplace(frac);

	// Orthogonalise it back
	return orthogonaliseInplace(frac);
}

template <typename FloatType>
inline typename AbstractFmidCell<FloatType>::Vec3 &
AbstractFmidCell<FloatType>::wrapVecInplace(typename AbstractFmidCell<FloatType>::Vec3 & toWrap) const
{
	// Fractionalise it
	toWrap = fractionaliseInplace(toWrap);

	// Wrap it
	wrapVecFracInplace(toWrap);

	// Orthogonalise it back
	toWrap = orthogonaliseInplace(toWrap);

	return toWrap;
}

template <typename FloatType>
inline typename AbstractFmidCell<FloatType>::Mat &
AbstractFmidCell<FloatType>::wrapVecsInplace(typename AbstractFmidCell::Mat & toWrap) const
{
	// Make sure we have three rows, otherwise this doesn't make sense
	SSE_ASSERT(toWrap.n_rows == 3);

	for(size_t i = 0; i < toWrap.n_cols; ++i)
	{
		typename AbstractFmidCell<FloatType>::Vec3 vec = toWrap.unsafe_col(i);
		wrapVecInplace(vec);
	}

	return toWrap;
}

template <typename FloatType>
typename AbstractFmidCell<FloatType>::Mat & AbstractFmidCell<FloatType>::wrapVecsInplaceFrac(typename AbstractFmidCell::Mat & fracs) const
{
	// Make sure we have three rows, otherwise this doesn't make sense
	SSE_ASSERT(fracs.n_rows == 3);

	for(size_t i = 0; i < fracs.n_cols; ++i)
	{
		fracs.at(0, i) -= floor(fracs.at(0, i));
		fracs.at(1, i) -= floor(fracs.at(1, i));
		fracs.at(2, i) -= floor(fracs.at(2, i));
	}

	return fracs;
}

template <typename FloatType>
typename AbstractFmidCell<FloatType>::Vec3 AbstractFmidCell<FloatType>::wrapVecFrac(const typename AbstractFmidCell<FloatType>::Vec3 & toWrap) const
{
	// Make a copy
	typename AbstractFmidCell<FloatType>::Vec3 wrapped = toWrap;
	// wrap it
	wrapVecFracInplace(wrapped);
	// give it back
	return wrapped;
}

template <typename FloatType>
typename AbstractFmidCell<FloatType>::Vec3 & AbstractFmidCell<FloatType>::wrapVecFracInplace(typename AbstractFmidCell<FloatType>::Vec3 & toWrap) const
{
	toWrap[0] -= floor(toWrap[0]);
	toWrap[1] -= floor(toWrap[1]);
	toWrap[2] -= floor(toWrap[2]);

	return toWrap;
}

template <typename FloatType>
typename AbstractFmidCell<FloatType>::Vec3 & AbstractFmidCell<FloatType>::fractionaliseInplace(typename AbstractFmidCell<FloatType>::Vec3 & cart) const
{
	// TODO: Look up Armadillo docs to see if there is a *= for post multiplication
	cart = myFracMtx * cart;
	return cart;
}

template <typename FloatType>
typename AbstractFmidCell<FloatType>::Mat & AbstractFmidCell<FloatType>::fractionaliseInplace(Mat & carts) const
{
	SSE_ASSERT(carts.n_rows == 3);

	// TODO: Look up Armadillo docs to see if there is a *= for post multiplication
	carts = myFracMtx * carts;
	return carts;
}


template <typename FloatType>
void AbstractFmidCell<FloatType>::getAllDistancesWithinCutoff(
	const typename AbstractFmidCell<FloatType>::Vec3 & cart1,
	const typename AbstractFmidCell<FloatType>::Vec3 & cart2,
	const FloatType cutoff,
	std::vector<FloatType> & outDistances,
	const size_t maxCellMultiples) const
{
	using namespace arma;
	using std::min;
	using std::max;

	// Make sure cart1 and 2 are in the unit cell at the origin
	//const ABSTRACT_FMID_CELL_TYPE::Vec3 cart1Unit = wrapVec(cart1);
	//const ABSTRACT_FMID_CELL_TYPE::Vec3 cart2Unit = wrapVec(cart2);
	const ABSTRACT_FMID_CELL_TYPE::Vec3	dR	= wrapVec(cart2) - wrapVec(cart1);

	// Get the lattice vectors
	const ABSTRACT_FMID_CELL_TYPE::Vec3 A = myOrthoMtx.col(0);
	const ABSTRACT_FMID_CELL_TYPE::Vec3 B = myOrthoMtx.col(1);
	const ABSTRACT_FMID_CELL_TYPE::Vec3 C = myOrthoMtx.col(2);

	// Maximum multiple of cell vectors we need to go to
	int maxA = (int)ceil(getNumPlaneRepetitionsToBoundSphere(A, B, C, cutoff));
	int maxB = (int)ceil(getNumPlaneRepetitionsToBoundSphere(B, A, C, cutoff));
	int maxC = (int)ceil(getNumPlaneRepetitionsToBoundSphere(C, A, B, cutoff));
	if(maxCellMultiples > 0)
	{
		maxA = min(maxA, (int)maxCellMultiples);
		maxB = min(maxB, (int)maxCellMultiples);
		maxC = min(maxC, (int)maxCellMultiples);
	}

	const FloatType	cutoffSq	= cutoff * cutoff;

	FloatType modDRSq;
	ABSTRACT_FMID_CELL_TYPE::Vec3 dFrac, dRImg;
	for(int a = -maxA; a <= maxA; ++a)
	{
		for(int b = -maxB; b <= maxB; ++b)
		{
			for(int c = -maxC; c <= maxC; ++c)
			{
				// TODO: Replace this with matrix multiplication
				dFrac[0] = a;
				dFrac[1] = b;
				dFrac[2] = c;

				dRImg = myOrthoMtx * dFrac + dR;
				modDRSq = dot(dRImg, dRImg);
				if(modDRSq < cutoffSq)
				{
					outDistances.push_back(sqrt(modDRSq));
				}
			}
		}
	}
}

template <typename FloatType>
inline typename AbstractFmidCell<FloatType>::Vec3 &
AbstractFmidCell<FloatType>::orthogonaliseInplace(typename AbstractFmidCell<FloatType>::Vec3 & frac) const
{
	frac = myOrthoMtx * frac;
	return frac;
}

template <typename FloatType>
inline typename AbstractFmidCell<FloatType>::Mat &
AbstractFmidCell<FloatType>::orthogonaliseInplace(Mat & fracs) const
{
	SSE_ASSERT(fracs.n_rows == 3);

	fracs = myOrthoMtx * fracs;
	return fracs;
}

template <typename FloatType>
FloatType AbstractFmidCell<FloatType>::getVolume() const
{
	return myVolume;
}

template <typename FloatType>
FloatType AbstractFmidCell<FloatType>::getNormVolume() const
{
  using arma::dot;
  using std::sqrt;

  // First normalise the lattice vectors
  typename AbstractFmidCell<FloatType>::Vec3 a = myOrthoMtx.col(0);
  typename AbstractFmidCell<FloatType>::Vec3 b = myOrthoMtx.col(1);
  typename AbstractFmidCell<FloatType>::Vec3 c = myOrthoMtx.col(2);

  a /= sqrt(dot(a, a));
  b /= sqrt(dot(b, b));
  c /= sqrt(dot(c, c));

  // Now calculate abs value of the the triple product
  return std::abs(dot(arma::cross(a, b), c));
}

template <typename FloatType>
FloatType AbstractFmidCell<FloatType>::setVolume(const FloatType volume)
{
	const FloatType scale = pow(volume / getVolume(), 1.0 / 3.0);

	init(scale * myOrthoMtx);

  return scale;
}

template <typename FloatType>
void AbstractFmidCell<FloatType>::init(
	const FloatType a, const FloatType b, const FloatType c,
	const FloatType alpha, const FloatType beta, const FloatType gamma)
{

	myLatticeParams[0] = a;
	myLatticeParams[1] = b;
	myLatticeParams[2] = c;
	myLatticeParams[3] = alpha;
	myLatticeParams[4] = beta;
	myLatticeParams[5] = gamma;
	
	initOrthoAndFracMatrices();
	initRest();
}

template <typename FloatType>
void AbstractFmidCell<FloatType>::init(const typename AbstractFmidCell<FloatType>::Mat33 & orthoMtx)
{
	myOrthoMtx	= orthoMtx;
	myFracMtx	= arma::inv(orthoMtx);

	initLatticeParams();
	initRest();
}

template <typename FloatType>
void AbstractFmidCell<FloatType>::initOrthoAndFracMatrices()
{
	// A - col 0
	myOrthoMtx.fill(0);
	myOrthoMtx.at(0, 0) = myLatticeParams[0];
	// B - col 1
	myOrthoMtx.at(0, 1) = myLatticeParams[1] * cos(Constants::DEG_TO_RAD * myLatticeParams[5]);
	myOrthoMtx.at(1, 1) = myLatticeParams[1] * sin(Constants::DEG_TO_RAD * myLatticeParams[5]);
	// C - col 2
	myOrthoMtx.at(0, 2) = myLatticeParams[2] * cos(Constants::DEG_TO_RAD * myLatticeParams[4]);
  // TODO: CHECK THIS THOUROUGHLY!!
	myOrthoMtx.at(1, 2) = myLatticeParams[2] * (cos(Constants::DEG_TO_RAD * myLatticeParams[3]) -
    cos(Constants::DEG_TO_RAD * myLatticeParams[4]) * cos(Constants::DEG_TO_RAD * myLatticeParams[5]))
    / sin(Constants::DEG_TO_RAD * myLatticeParams[5]);
	myOrthoMtx.at(2, 2) = sqrt(myLatticeParams[2] * myLatticeParams[2] -
		myOrthoMtx(0, 2) * myOrthoMtx(0, 2) -
		myOrthoMtx(1, 2) * myOrthoMtx(1, 2));

	myFracMtx = inv(myOrthoMtx);
}

template <typename FloatType>
void AbstractFmidCell<FloatType>::initLatticeParams()
{
	using namespace arma;

	// Get the lattice vectors
	const typename AbstractFmidCell<FloatType>::Vec3 a = myOrthoMtx.col(0);
	const typename AbstractFmidCell<FloatType>::Vec3 b = myOrthoMtx.col(1);
	const typename AbstractFmidCell<FloatType>::Vec3 c = myOrthoMtx.col(2);

	myLatticeParams[0] = std::sqrt(dot(a, a));
	myLatticeParams[1] = std::sqrt(dot(b, b));
	myLatticeParams[2] = std::sqrt(dot(c, c));
	myLatticeParams[3] = acos(dot(b, c) / (myLatticeParams[1] * myLatticeParams[2])) * Constants::RAD_TO_DEG;
	myLatticeParams[4] = acos(dot(a, c) / (myLatticeParams[0] * myLatticeParams[2])) * Constants::RAD_TO_DEG;
	myLatticeParams[5] = acos(dot(a, b) / (myLatticeParams[0] * myLatticeParams[1])) * Constants::RAD_TO_DEG;
}

template <typename FloatType>
void AbstractFmidCell<FloatType>::initRest()
{
	myVolume = std::fabs(
		dot(myOrthoMtx.col(0),
		cross(myOrthoMtx.col(1), myOrthoMtx.col(2))));
}

// Borrowed from David Lonie's XtalOpt here:
// http://github.com/dlonie/XtalComp/blob/master/xtalcomp.cpp#L1538
//
// Implements the niggli reduction algorithm detailed in:
// Grosse-Kunstleve RW, Sauter NK, Adams PD. Numerically stable
// algorithms for the computation of reduced unit cells. Acta
// Crystallographica Section A Foundations of
// Crystallography. 2003;60(1):1-6.
template <typename FloatType>
bool AbstractFmidCell<FloatType>::niggliReduce()
{
  using namespace sstbx::utility;
  using std::fabs;

  // Set maximum number of iterations
  const unsigned int iterations = 1000;

  // For sanity checks:
  const double origVolume = getVolume();

  // Cache the current fractional coordinates for later.
  //QList<Eigen::Vector3d> fcoords = currentFractionalCoords();

  // Get cell parameters in storage units, convert deg->rad
  double a     = myLatticeParams[0];
  double b     = myLatticeParams[1];
  double c     = myLatticeParams[2];
  double alpha = myLatticeParams[3] * Constants::DEG_TO_RAD;
  double beta  = myLatticeParams[4] * Constants::DEG_TO_RAD;
  double gamma = myLatticeParams[5] * Constants::DEG_TO_RAD;

  // Compute characteristic (step 0)
  double A    = a*a;
  double B    = b*b;
  double C    = c*c;
  double xi   = 2*b*c*cos(alpha);
  double eta  = 2*a*c*cos(beta);
  double zeta = 2*a*b*cos(gamma);

  // Return value
  bool ret = false;

  // comparison tolerance
  const double tol = StableComp::STABLE_COMP_TOL * std::pow(a * b * c, 1.0/3.0);

  // Initialize change of basis matrices:
  //
  // Although the reduction algorithm produces quantities directly
  // relatible to a,b,c,alpha,beta,gamma, we will calculate a change
  // of basis matrix to use instead, and discard A, B, C, xi, eta,
  // zeta. By multiplying the change of basis matrix against the
  // current cell matrix, we avoid the problem of handling the
  // orientation matrix already present in the cell. The inverse of
  // this matrix can also be used later to convert the atomic
  // positions.
  // tmpMat is used to build other matrices
  arma::mat33 tmpMat;

  // Cache static matrices:

  // Swap x,y (Used in Step 1). Negatives ensure proper sign of final
  // determinant.
  tmpMat
    << 0 << -1 << 0 << arma::endr
    << -1 << 0 << 0 << arma::endr
    << 0 << 0 << -1 << arma::endr;

  const arma::mat33 C1(tmpMat);
  // Swap y,z (Used in Step 2). Negatives ensure proper sign of final
  // determinant
  tmpMat
    << -1 << 0 << 0 << arma::endr
    << 0 << 0 << -1 << arma::endr
    << 0 << -1 << 0 << arma::endr;
  const arma::mat33 C2(tmpMat);
  // For step 8:
  tmpMat
    << 1 << 0 << 1 << arma::endr
    << 0 << 1 << 1 << arma::endr
    << 0 << 0 << 1 << arma::endr;
  const arma::mat33 C8(tmpMat);

  // initial change of basis matrix
  tmpMat
    << 1 << 0 << 0 << arma::endr
    << 0 << 1 << 0 << arma::endr
    << 0 << 0 << 1 << arma::endr;
  arma::mat33 cob(tmpMat);

//#define NIGGLI_DEBUG(step) std::cout << iter << " " << step << " " << A << " "\
//  << B << " " << C << " " << xi << " " << eta << " " << zeta << std::endl;
#define NIGGLI_DEBUG(step)
  unsigned int iter;
  for (iter = 0; iter < iterations; ++iter) {
    // Step 1:
    if (
        StableComp::gt(A, B, tol)
        || (
            StableComp::eq(A, B, tol)
            &&
            StableComp::gt(fabs(xi), fabs(eta), tol)
            )
        ) {
      cob *= C1;
      std::swap(A, B);
      std::swap(xi, eta);
      NIGGLI_DEBUG(1);
    }

    // Step 2:
    if (
        StableComp::gt(B, C, tol)
        || (
            StableComp::eq(B, C, tol)
            &&
            StableComp::gt(fabs(eta), fabs(zeta), tol)
            )
        ) {
      cob *= C2;
      std::swap(B, C);
      std::swap(eta, zeta);
      NIGGLI_DEBUG(2);
      continue;
    }

    // Step 3:
    // Use exact comparisons in steps 3 and 4.
    if (xi*eta*zeta > 0) {
      // Update change of basis matrix:
      tmpMat
        << StableComp::sign(xi) << 0 << 0 << arma::endr
        << 0 << StableComp::sign(eta) << 0 << arma::endr
        << 0 << 0 << StableComp::sign(zeta) << arma::endr;
      cob *= tmpMat;

      // Update characteristic
      xi   = fabs(xi);
      eta  = fabs(eta);
      zeta = fabs(zeta);
      NIGGLI_DEBUG(3);
      ++iter;
    }

    // Step 4:
    // Use exact comparisons for steps 3 and 4
    else { // either step 3 or 4 should run
      // Update change of basis matrix:
      double *p = NULL;
      double i = 1;
      double j = 1;
      double k = 1;
      if (xi > 0) {
        i = -1;
      }
      else if (!(xi < 0)) {
        p = &i;
      }
      if (eta > 0) {
        j = -1;
      }
      else if (!(eta < 0)) {
        p = &j;
      }
      if (zeta > 0) {
        k = -1;
      }
      else if (!(zeta < 0)) {
        p = &k;
      }
      if (i*j*k < 0) {
        if (!p) {
          //QMessageBox::warning
          //    (m_mainwindow,
          //     CE_DIALOG_TITLE,
          //     tr("Niggli-reduction failed. The input structure's "
          //        "lattice that is confusing the Niggli-reduction "
          //        "algorithm. Try making a small perturbation (approx."
          //        " 2 orders of magnitude smaller than the tolerance) "
          //        "to the input lattices and try again."));
          return false;
        }
        *p = -1;
      }
      tmpMat
        << i << 0 << 0 << arma::endr
        << 0 << j << 0 << arma::endr
        << 0 << 0 << k << arma::endr;
      cob *= tmpMat;

      // Update characteristic
      xi   = -fabs(xi);
      eta  = -fabs(eta);
      zeta = -fabs(zeta);
      NIGGLI_DEBUG(4);
      ++iter;
    }

    // Step 5:
    if (StableComp::gt(fabs(xi), B, tol)
        || (StableComp::eq(xi, B, tol)
            && StableComp::lt(2*eta, zeta, tol)
            )
        || (StableComp::eq(xi, -B, tol)
            && StableComp::lt(zeta, 0, tol)
            )
        ) {
      double signXi = StableComp::sign(xi);
      // Update change of basis matrix:
      tmpMat
        << 1 << 0 << 0 << arma::endr
        << 0 << 1 << -signXi << arma::endr
        << 0 << 0 << 1 << arma::endr;
      cob *= tmpMat;

      // Update characteristic
      C    = B + C - xi*signXi;
      eta  = eta - zeta*signXi;
      xi   = xi -   2*B*signXi;
      NIGGLI_DEBUG(5);
      continue;
    }

    // Step 6:
    if (StableComp::gt(fabs(eta), A, tol)
        || (StableComp::eq(eta, A, tol)
            && StableComp::lt(2*xi, zeta, tol)
            )
        || (StableComp::eq(eta, -A, tol)
            && StableComp::lt(zeta, 0, tol)
            )
        ) {
      double signEta = StableComp::sign(eta);
      // Update change of basis matrix:
      tmpMat
        << 1 << 0 << -signEta << arma::endr
        << 0 << 1 << 0 << arma::endr
        << 0 << 0 << 1 << arma::endr;
      cob *= tmpMat;

      // Update characteristic
      C    = A + C - eta*signEta;
      xi   = xi - zeta*signEta;
      eta  = eta - 2*A*signEta;
      NIGGLI_DEBUG(6);
      continue;
    }

    // Step 7:
    if (StableComp::gt(fabs(zeta), A, tol)
        || (StableComp::eq(zeta, A, tol)
            && StableComp::lt(2*xi, eta, tol)
            )
        || (StableComp::eq(zeta, -A, tol)
            && StableComp::lt(eta, 0, tol)
            )
        ) {
      double signZeta = StableComp::sign(zeta);
      // Update change of basis matrix:
      tmpMat
        << 1 << -signZeta << 0 << arma::endr
        << 0 << 1 << 0 << arma::endr
        << 0 << 0 << 1 << arma::endr;
      cob *= tmpMat;

      // Update characteristic
      B    = A + B - zeta*signZeta;
      xi   = xi - eta*signZeta;
      zeta = zeta - 2*A*signZeta;
      NIGGLI_DEBUG(7);
      continue;
    }

    // Step 8:
    double sumAllButC = A + B + xi + eta + zeta;
    if (StableComp::lt(sumAllButC, 0, tol)
        || (StableComp::eq(sumAllButC, 0, tol)
            && StableComp::gt(2*(A+eta)+zeta, 0, tol)
            )
        ) {
      // Update change of basis matrix:
      cob *= C8;

      // Update characteristic
      C    = sumAllButC + C;
      xi = 2*B + xi + zeta;
      eta  = 2*A + eta + zeta;
      NIGGLI_DEBUG(8);
      continue;
    }

    // Done!
    ret = true;
    break;
  }

  // No change
  if (iter == 0) {
    //QMessageBox::information
    //  (m_mainwindow,
    //   CE_DIALOG_TITLE,
    //   tr("This unit cell is already reduced to "
    //      "its canonical Niggli representation."));
    return false;
  }

  if (!ret) {
    //QMessageBox::warning
    //  (m_mainwindow,
    //   CE_DIALOG_TITLE,
    //   tr("Failed to reduce cell after 1000 iterations of "
    //      "the reduction algorithm. Stopping."));
    return false;
  }

  SSE_ASSERT(arma::det(cob) == 1);

  //Q_ASSERT_X(cob.determinant() == 1, Q_FUNC_INFO,
  //           "Determinant of change of basis matrix must be 1.");

  // Update cell
  init(myOrthoMtx * cob);
  //setCurrentCellMatrix(cob.transpose() * currentCellMatrix());

  //Q_ASSERT_X(StableComp::eq(origVolume, currentVolume(), tol),
  //           Q_FUNC_INFO, "Cell volume changed during Niggli reduction.");

  // fix coordinates
  // Apply COB matrix:
 /* Eigen::Matrix3d invCob;
  cob.computeInverse(&invCob);
  for (QList<Eigen::Vector3d>::iterator
         it = fcoords.begin(),
         it_end = fcoords.end();
       it != it_end; ++it) {
    *it = invCob * (*it);
  }
  setCurrentFractionalCoords(currentAtomicSymbols(), fcoords);*/

  // wrap:
  //wrapAtomsToCell();
  //orientStandard();
  return true;
}

template <typename FloatType>
FloatType AbstractFmidCell<FloatType>::getLongestVector() const
{
	return std::max(myLatticeParams[0], std::max(myLatticeParams[1], myLatticeParams[2]));	
}

template <typename FloatType>
FloatType AbstractFmidCell<FloatType>::getLongestVectorSq() const
{
	const FloatType longest = getLongestVector();
	return longest * longest;
}

}} // Close the namespace

#endif /* ABSTRACT_FMID_CELL_H */
