/*
 * AbstractFmidCell.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES ///////////////

#include "SSLibAssert.h"
#include "common/AbstractFmidCell.h"
#include "utility/StableComparison.h"

namespace sstbx
{
namespace common
{



AbstractFmidCell::AbstractFmidCell(const AbstractFmidCell & toCopy)
{
	init(toCopy.getOrthoMtx());
}



AbstractFmidCell::AbstractFmidCell(
	const double a, const double b, const double c,
	const double alpha, const double beta, const double gamma)
{
	init(a, b, c, alpha, beta, gamma);
}


AbstractFmidCell::AbstractFmidCell(const double (&latticeParams)[6])
{
	// TODO: Tidy this up a bit
	init(latticeParams[0], latticeParams[1], latticeParams[2],
		latticeParams[3], latticeParams[4], latticeParams[5]);
}


double AbstractFmidCell::getDistanceMinimumImg(
	const AbstractFmidCell::Vec3 & a, const AbstractFmidCell::Vec3 & b) const
{
	return sqrt(getDistanceSqMinimumImg(a, b));
}


double AbstractFmidCell::getDistanceSqMinimumImg(
	const AbstractFmidCell::Vec3 & a,
	const AbstractFmidCell::Vec3 & b) const
{
	using namespace arma;
	AbstractFmidCell::Vec3 dr = getVecMinimumImg(a, b);
	return dot(dr, dr);
}


const double (&AbstractFmidCell::getLatticeParams() const)[6]
{
	return myLatticeParams;
}


AbstractFmidCell::Vec3 AbstractFmidCell::getVecMinimumImg(
	const AbstractFmidCell::Vec3 & a,
	const AbstractFmidCell::Vec3 & b) const
{
	using namespace arma;

	// Make sure cart1 and 2 are in the unit cell at the origin
	const AbstractFmidCell::Vec3 		aUnit	= wrapVec(a);
	const AbstractFmidCell::Vec3 		bUnit	= wrapVec(b);
	const AbstractFmidCell::Vec3		dR		= bUnit - aUnit;
	double		minModDRSq= dot(dR, dR);
	const double minModDR= sqrt(minModDRSq);

	// Get the lattice vectors
	const AbstractFmidCell::Vec3 A = myOrthoMtx.col(0);
	const AbstractFmidCell::Vec3 B = myOrthoMtx.col(1);
	const AbstractFmidCell::Vec3 C = myOrthoMtx.col(2);

	// Maximum multiple of cell vectors we need to go to
	int maxA = (int)ceil(getNumPlaneRepetitionsToBoundSphere(A, B, C, minModDR));
	int maxB = (int)ceil(getNumPlaneRepetitionsToBoundSphere(B, A, C, minModDR));
	int maxC = (int)ceil(getNumPlaneRepetitionsToBoundSphere(C, A, B, minModDR));

	AbstractFmidCell::Vec3 dFrac, dRImg, minDR = dR;
	double modDRSq;
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


void AbstractFmidCell::getAllVectorsWithinCutoff(
	const AbstractFmidCell::Vec3 & cart1,
	const AbstractFmidCell::Vec3 & cart2,
	const double cutoff,
	std::vector<AbstractFmidCell::Vec3> & outVectors,
  const size_t maxVectors) const
{
	using namespace arma;

	// Make sure cart1 and 2 are in the unit cell at the origin
	const AbstractFmidCell::Vec3 cart1Unit = wrapVec(cart1);
	const AbstractFmidCell::Vec3 cart2Unit = wrapVec(cart2);

	// Get the lattice vectors
	const AbstractFmidCell::Vec3 A = myOrthoMtx.col(0);
	const AbstractFmidCell::Vec3 B = myOrthoMtx.col(1);
	const AbstractFmidCell::Vec3 C = myOrthoMtx.col(2);

	// Maximum multiple of cell vectors we need to go to
	int maxA = (int)ceil(getNumPlaneRepetitionsToBoundSphere(A, B, C, cutoff));
	int maxB = (int)ceil(getNumPlaneRepetitionsToBoundSphere(B, A, C, cutoff));
	int maxC = (int)ceil(getNumPlaneRepetitionsToBoundSphere(C, A, B, cutoff));

	const AbstractFmidCell::Vec3 dR = cart2Unit - cart1Unit;
	const double	cutoffSq	= cutoff * cutoff;

	AbstractFmidCell::Vec3 dFrac, dRImg;
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

double AbstractFmidCell::getNumPlaneRepetitionsToBoundSphere(
	const typename arma::Row<double>::fixed<3> & boundDir,
	const typename arma::Row<double>::fixed<3> & planeVecA,
	const typename arma::Row<double>::fixed<3> & planeVecB,
	const double radius) const
{
	// The vector normal to the plane
	const AbstractFmidCell::Vec3 normal = arma::cross(planeVecA, planeVecB);

	return radius / getVolume() * std::sqrt(arma::dot(normal, normal));
}
*/


double AbstractFmidCell::getNumPlaneRepetitionsToBoundSphere(
	const AbstractFmidCell::Vec3 & boundDir,
	const AbstractFmidCell::Vec3 & planeVecA,
	const AbstractFmidCell::Vec3 & planeVecB,
	const double radius) const
{
	// The vector normal to the plane
	const AbstractFmidCell::Vec3 normal = arma::cross(planeVecA, planeVecB);

	return radius / getVolume() * std::sqrt(arma::dot(normal, normal));
}


void AbstractFmidCell::getAllDistancesBetweenPoints(
  const AbstractFmidCell::Mat & points,
  AbstractFmidCell::Mat & distancesOut
) const
{
  SSE_ASSERT(points.n_rows == 3);

  const size_t numPoints = points.n_cols;

  distancesOut.set_size(numPoints, numPoints);

  for(size_t i = 0; i < numPoints - 1; ++i)
  {
    for(size_t j = i + 1; j < numPoints; ++j)
    {
      distancesOut(i, j) = getDistanceMinimumImg(points.col(i), points.col(j));
    }
  }
  distancesOut = ::arma::symmatu(distancesOut);
}


AbstractFmidCell::Vec3 AbstractFmidCell::randomPoint() const
{
	// Generate a random point in fractional coords and then
	// orthogonalise it
	AbstractFmidCell::Vec3 x;
	x.randu();
	return orthogonaliseInplace(x);
}


inline AbstractFmidCell::Vec3
AbstractFmidCell::fractionalise(const AbstractFmidCell::Vec3 & cartesian) const
{
	return myFracMtx * cartesian;
}


inline AbstractFmidCell::Vec3
AbstractFmidCell::orthogonalise(const AbstractFmidCell::Vec3 & frac) const
{
	return myOrthoMtx * frac;
}


const AbstractFmidCell::Mat33 & AbstractFmidCell::getOrthoMtx() const
{
	return myOrthoMtx;
}


void AbstractFmidCell::setOrthoMtx(const AbstractFmidCell::Mat33 & orthoMtx)
{
	myOrthoMtx = orthoMtx;
	init(orthoMtx);
}


const AbstractFmidCell::Mat33 AbstractFmidCell::getFracMtx() const
{
	return myFracMtx;
}


AbstractFmidCell::Vec3 AbstractFmidCell::wrapVec(const AbstractFmidCell::Vec3 & toWrap) const
{
	using namespace arma;

	// Fractionalise it
	AbstractFmidCell::Vec3 frac = myFracMtx * toWrap;

	// Wrap it
	wrapVecFracInplace(frac);

	// Orthogonalise it back
	return orthogonaliseInplace(frac);
}


inline AbstractFmidCell::Vec3 &
AbstractFmidCell::wrapVecInplace(AbstractFmidCell::Vec3 & toWrap) const
{
	// Fractionalise it
	toWrap = fractionaliseInplace(toWrap);

	// Wrap it
	wrapVecFracInplace(toWrap);

	// Orthogonalise it back
	toWrap = orthogonaliseInplace(toWrap);

	return toWrap;
}


AbstractFmidCell::Mat &
AbstractFmidCell::wrapVecsInplace(AbstractFmidCell::Mat & toWrap) const
{
	// Make sure we have three rows, otherwise this doesn't make sense
	SSE_ASSERT(toWrap.n_rows == 3);

	for(size_t i = 0; i < toWrap.n_cols; ++i)
	{
		AbstractFmidCell::Vec3 vec = toWrap.unsafe_col(i);
		wrapVecInplace(vec);
	}

	return toWrap;
}


AbstractFmidCell::Mat &
AbstractFmidCell::wrapVecsInplaceFrac(AbstractFmidCell::Mat & fracs) const
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


AbstractFmidCell::Vec3 AbstractFmidCell::wrapVecFrac(const AbstractFmidCell::Vec3 & toWrap) const
{
	// Make a copy
	AbstractFmidCell::Vec3 wrapped = toWrap;
	// wrap it
	wrapVecFracInplace(wrapped);
	// give it back
	return wrapped;
}


AbstractFmidCell::Vec3 & AbstractFmidCell::wrapVecFracInplace(AbstractFmidCell::Vec3 & toWrap) const
{
	toWrap[0] -= floor(toWrap[0]);
	toWrap[1] -= floor(toWrap[1]);
	toWrap[2] -= floor(toWrap[2]);

	return toWrap;
}


AbstractFmidCell::Vec3 & AbstractFmidCell::fractionaliseInplace(AbstractFmidCell::Vec3 & cart) const
{
	// TODO: Look up Armadillo docs to see if there is a *= for post multiplication
	cart = myFracMtx * cart;
	return cart;
}


AbstractFmidCell::Mat & AbstractFmidCell::fractionaliseInplace(Mat & carts) const
{
	SSE_ASSERT(carts.n_rows == 3);

	// TODO: Look up Armadillo docs to see if there is a *= for post multiplication
	carts = myFracMtx * carts;
	return carts;
}



void AbstractFmidCell::getAllDistancesWithinCutoff(
	const AbstractFmidCell::Vec3 & cart1,
	const AbstractFmidCell::Vec3 & cart2,
	const double cutoff,
	std::vector<double> & outDistances,
	const size_t maxCellMultiples) const
{
	using namespace arma;
	using std::min;
	using std::max;

	// Make sure cart1 and 2 are in the unit cell at the origin
	//const AbstractFmidCell::Vec3 cart1Unit = wrapVec(cart1);
	//const AbstractFmidCell::Vec3 cart2Unit = wrapVec(cart2);
	const AbstractFmidCell::Vec3	dR	= wrapVec(cart2) - wrapVec(cart1);

	// Get the lattice vectors
	const AbstractFmidCell::Vec3 A = myOrthoMtx.col(0);
	const AbstractFmidCell::Vec3 B = myOrthoMtx.col(1);
	const AbstractFmidCell::Vec3 C = myOrthoMtx.col(2);

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

	const double	cutoffSq	= cutoff * cutoff;

	double modDRSq;
	AbstractFmidCell::Vec3 dFrac, dRImg;
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


inline AbstractFmidCell::Vec3 &
AbstractFmidCell::orthogonaliseInplace(AbstractFmidCell::Vec3 & frac) const
{
	frac = myOrthoMtx * frac;
	return frac;
}


inline AbstractFmidCell::Mat &
AbstractFmidCell::orthogonaliseInplace(Mat & fracs) const
{
	SSE_ASSERT(fracs.n_rows == 3);

	fracs = myOrthoMtx * fracs;
	return fracs;
}


double AbstractFmidCell::getVolume() const
{
	return myVolume;
}


double AbstractFmidCell::getNormVolume() const
{
  using arma::dot;
  using std::sqrt;

  // First normalise the lattice vectors
  AbstractFmidCell::Vec3 a = myOrthoMtx.col(0);
  AbstractFmidCell::Vec3 b = myOrthoMtx.col(1);
  AbstractFmidCell::Vec3 c = myOrthoMtx.col(2);

  a /= sqrt(dot(a, a));
  b /= sqrt(dot(b, b));
  c /= sqrt(dot(c, c));

  // Now calculate abs value of the the triple product
  return std::abs(dot(arma::cross(a, b), c));
}


double AbstractFmidCell::setVolume(const double volume)
{
	const double scale = pow(volume / getVolume(), 1.0 / 3.0);

  // TEST;
  if(scale != scale)
  {
    ::std::cout << "GOT HERE";
  }

	init(scale * myOrthoMtx);

  return scale;
}


void AbstractFmidCell::init(
	const double a, const double b, const double c,
	const double alpha, const double beta, const double gamma)
{
  // Sanity checks on parameters
  SSLIB_ASSERT_MSG(alpha+beta+gamma <= 360.0, "Non-physical lattice parameters supplied - required that alpha+beta+gamma <= 360.0");
  SSLIB_ASSERT_MSG(abs(alpha-beta) <= gamma, "Non-physical lattice parameters supplied - require that abs(alpha-beta) < gamma");
  SSLIB_ASSERT_MSG(abs(beta-gamma) <= alpha, "Non-physical lattice parameters supplied - require that abs(beta-gamma) < alpha");
  SSLIB_ASSERT_MSG(abs(gamma-alpha) <= beta, "Non-physical lattice parameters supplied - require that abs(gamma-alpha) < beta");

	myLatticeParams[0] = a;
	myLatticeParams[1] = b;
	myLatticeParams[2] = c;
	myLatticeParams[3] = alpha;
	myLatticeParams[4] = beta;
	myLatticeParams[5] = gamma;
	
	initOrthoAndFracMatrices();
	initRest();
}


void AbstractFmidCell::init(const AbstractFmidCell::Mat33 & orthoMtx)
{
	myOrthoMtx	= orthoMtx;
	myFracMtx	= arma::inv(orthoMtx);

	initLatticeParams();
	initRest();
}


void AbstractFmidCell::initOrthoAndFracMatrices()
{
  const double alphaRad = Constants::DEG_TO_RAD * myLatticeParams[3];
  const double betaRad = Constants::DEG_TO_RAD * myLatticeParams[4];
  const double gammaRad = Constants::DEG_TO_RAD * myLatticeParams[5];

	myOrthoMtx.fill(0);
	// A - col 0
	myOrthoMtx.at(0, 0) = myLatticeParams[0];
	// B - col 1
	myOrthoMtx.at(0, 1) = myLatticeParams[1] * cos(gammaRad);
	myOrthoMtx.at(1, 1) = myLatticeParams[1] * sin(gammaRad);
	// C - col 2
	myOrthoMtx.at(0, 2) = myLatticeParams[2] * cos(betaRad);
	myOrthoMtx.at(1, 2) = myLatticeParams[2] * (cos(alphaRad) - cos(betaRad) * cos(gammaRad)) / sin(gammaRad);
	myOrthoMtx.at(2, 2) = sqrt(
    myLatticeParams[2] * myLatticeParams[2] -
		myOrthoMtx(0, 2) * myOrthoMtx(0, 2) -
		myOrthoMtx(1, 2) * myOrthoMtx(1, 2)
  );

	myFracMtx = inv(myOrthoMtx);
}


void AbstractFmidCell::initLatticeParams()
{
	using namespace arma;

	// Get the lattice vectors
	const AbstractFmidCell::Vec3 a = myOrthoMtx.col(0);
	const AbstractFmidCell::Vec3 b = myOrthoMtx.col(1);
	const AbstractFmidCell::Vec3 c = myOrthoMtx.col(2);

	myLatticeParams[0] = std::sqrt(dot(a, a));
	myLatticeParams[1] = std::sqrt(dot(b, b));
	myLatticeParams[2] = std::sqrt(dot(c, c));
	myLatticeParams[3] = acos(dot(b, c) / (myLatticeParams[1] * myLatticeParams[2])) * Constants::RAD_TO_DEG;
	myLatticeParams[4] = acos(dot(a, c) / (myLatticeParams[0] * myLatticeParams[2])) * Constants::RAD_TO_DEG;
	myLatticeParams[5] = acos(dot(a, b) / (myLatticeParams[0] * myLatticeParams[1])) * Constants::RAD_TO_DEG;
}


void AbstractFmidCell::initRest()
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

bool AbstractFmidCell::niggliReduce()
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


double AbstractFmidCell::getLongestVector() const
{
	return std::max(myLatticeParams[0], std::max(myLatticeParams[1], myLatticeParams[2]));	
}


double AbstractFmidCell::getLongestVectorSq() const
{
	const double longest = getLongestVector();
	return longest * longest;
}

}
}
