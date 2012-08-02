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

#include <armadillo>

// Local includes
#include "common/Constants.h"


// DEFINES ////////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
namespace common
{

class AbstractFmidCell
{
public:

	// TYPEDEFS ////////////////////////
	// Need to use the typename keyword to make it clear that these are types
	typedef arma::Mat<double>				Mat;
	typedef arma::Mat<double>::fixed<3, 3>	Mat33;
	typedef arma::Col<double>::fixed<3>		Vec3;

  static const unsigned int MAX_OUT_VECTORS = 100000;

	// Copy constructor
	AbstractFmidCell(const AbstractFmidCell & toCopy);

	AbstractFmidCell(
		const double a, const double b, const double c,
		const double alpha, const double beta, const double gamma);

	explicit AbstractFmidCell(const double (&latticeParams)[6]);

	/**
	/* Get the minimum distance between two cartesian points respecting the boundary
	/* conditions
	/**/
	virtual double getDistanceMinimumImg(
		const AbstractFmidCell::Vec3 & a,
		const AbstractFmidCell::Vec3 & b) const;

	virtual double getDistanceSqMinimumImg(
		const AbstractFmidCell::Vec3 & a,
		const AbstractFmidCell::Vec3 & b) const;

	const double (&getLatticeParams() const)[6];

	virtual AbstractFmidCell::Vec3 getVecMinimumImg(
		const AbstractFmidCell::Vec3 & a,
		const AbstractFmidCell::Vec3 & b) const;

	virtual void getAllVectorsWithinCutoff(
		const AbstractFmidCell::Vec3 & cart1,
		const AbstractFmidCell::Vec3 & cart2,
		const double cutoff,
		std::vector<AbstractFmidCell::Vec3> & outVectors,
    const size_t    maxVectors = MAX_OUT_VECTORS) const;

	virtual void getAllDistancesWithinCutoff(
		const AbstractFmidCell::Vec3 & cart1,
		const AbstractFmidCell::Vec3 & cart2,
		const double cutoff,
		std::vector<double> & outVectors,
		const size_t maxCellMultiples = 0) const;

	double getNumPlaneRepetitionsToBoundSphere(
		const AbstractFmidCell::Vec3 & boundDir,
		const AbstractFmidCell::Vec3 & planeVecA,
		const AbstractFmidCell::Vec3 & planeVecB,
		const double radius) const;

  virtual void getAllDistancesBetweenPoints(
    const AbstractFmidCell::Mat & points,
    AbstractFmidCell::Mat & distancesOut
  ) const;

	AbstractFmidCell::Vec3 wrapVec(const AbstractFmidCell::Vec3 & toWrap) const;
	AbstractFmidCell::Vec3 & wrapVecInplace(AbstractFmidCell::Vec3 & toWrap) const;

	AbstractFmidCell::Mat & wrapVecsInplace(AbstractFmidCell::Mat & carts) const;
	AbstractFmidCell::Mat & wrapVecsInplaceFrac(AbstractFmidCell::Mat & fracs) const;

	AbstractFmidCell::Vec3 wrapVecFrac(const AbstractFmidCell::Vec3 & toWrap) const;
	AbstractFmidCell::Vec3 & wrapVecFracInplace(AbstractFmidCell::Vec3 & toWrap) const;

	virtual Vec3 randomPoint() const;

	/////////////////////////////////////////////////////////
	// VECTOR TRANSFORMATION METHODS ////////////////////////

	virtual Vec3 fractionalise(const AbstractFmidCell::Vec3 & cartesian) const;

	virtual Vec3 & fractionaliseInplace(AbstractFmidCell::Vec3 & cart) const;

	/**
	/* Fractionalise multiple cartesian vectors.  This method takes an argument that
	/* must be a 3 x n matrix.
	/**/
	virtual Mat & fractionaliseInplace(AbstractFmidCell::Mat & carts) const;

	virtual Vec3 orthogonalise(const AbstractFmidCell::Vec3 & frac) const;

	virtual Vec3 & orthogonaliseInplace(AbstractFmidCell::Vec3 & frac) const;

	virtual Mat & orthogonaliseInplace(AbstractFmidCell::Mat & fracs) const;

	// END ORTHOGONALISATION METHODS //////////////////////////////

	virtual const Mat33 & getOrthoMtx() const;

	virtual void setOrthoMtx(const AbstractFmidCell::Mat33 & orthoMtx);
	const Mat33 getFracMtx() const;

	// TODO: RENAME THESE!
	double getLongestVector() const;
	double getLongestVectorSq() const;

	virtual double getVolume() const;
  
  double getNormVolume() const;

	double setVolume(const double volume);

  bool niggliReduce();

protected:

	/** Initialise the unit cell from lattice parameters */
	void init(
		const double a, const double b, const double c,
		const double alpha, const double beta, const double gamma);
	/** Initialise the unit cell from an orthogonalisation matrix */
	void init(const Mat33 & orthoMtx);
	void initOrthoAndFracMatrices();
	void initLatticeParams();
	void initRest();

	/** The unit cell matrix where columns represent basis vectors */
	AbstractFmidCell::Mat33 myOrthoMtx;

	/** The inverse of the orthogonalisation matrix */
	AbstractFmidCell::Mat33 myFracMtx;

	double	myLatticeParams[6];

	double	myVolume;
};

}
} // Close the namespace

#endif /* ABSTRACT_FMID_CELL_H */
