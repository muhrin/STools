/*
 * UniversalCrystalDistanceCalculator.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES ///////////////
#include "common/UniversalCrystalDistanceCalculator.h"

#include "common/Structure.h"
#include "common/UnitCell.h"


namespace sstbx {
namespace common {

UniversalCrystalDistanceCalculator::UniversalCrystalDistanceCalculator(const Structure & structure):
DistanceCalculator(structure)
{}

::arma::vec3 UniversalCrystalDistanceCalculator::getVecMinImg(const ::arma::vec3 & a, const ::arma::vec3 & b) const
{
  const UnitCell & cell = *myStructure.getUnitCell();

	// Make sure cart1 and 2 are in the unit cell at the origin
  const ::arma::vec3 		aUnit	= cell.wrapVec(a);
  const ::arma::vec3 		bUnit	= cell.wrapVec(b);
  const ::arma::vec3		dR		= bUnit - aUnit;
	double minModDRSq = dot(dR, dR);
	const double minModDR = sqrt(minModDRSq);

	// Get the lattice vectors
  const ::arma::vec3 A(cell.getAVec());
  const ::arma::vec3 B(cell.getBVec());
  const ::arma::vec3 C(cell.getCVec());

	// Maximum multiple of cell vectors we need to go to
	int maxA = (int)ceil(getNumPlaneRepetitionsToBoundSphere(A, B, C, minModDR));
	int maxB = (int)ceil(getNumPlaneRepetitionsToBoundSphere(B, A, C, minModDR));
	int maxC = (int)ceil(getNumPlaneRepetitionsToBoundSphere(C, A, B, minModDR));

  // Loop variables
  ::arma::vec3 minDR = dR;
  const ::arma::mat33 & orthoMtx = cell.getOrthoMtx();
	double modDRSq;
  ::arma::vec3 nA, nAPlusNB, dRImg;
  size_t numDistances = 0;
	for(int a = -maxA; a <= maxA; ++a)
	{
		nA = a * A;
		for(int b = -maxB; b <= maxB; ++b)
		{
		  nAPlusNB = nA + b * B;
			for(int c = -maxC; c <= maxC; ++c)
			{
        dRImg = nAPlusNB + c * C + dR;
				
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

bool UniversalCrystalDistanceCalculator::getDistsBetween(
    const ::arma::vec3 & a,
    const ::arma::vec3 & b,
    const double cutoff,
    ::std::vector<double> & outValues,
    const size_t maxValues) const
{
  const UnitCell & cell = *myStructure.getUnitCell();
  const double (&params)[6] = cell.getLatticeParams();

	// Make sure a and b are in the unit cell at the origin
  const ::arma::vec3		dR		= cell.wrapVec(b) - cell.wrapVec(a);

	// Get the lattice vectors
  const ::arma::vec3 A(cell.getAVec());
  const ::arma::vec3 B(cell.getBVec());
  const ::arma::vec3 C(cell.getCVec());

  const double safeCutoff = cutoff + sqrt(::arma::dot(dR, dR));

  const double rDotA = ::arma::dot(dR, A / params[0]);
  const double rDotB = ::arma::dot(dR, B / params[1]);
  const double rDotC = ::arma::dot(dR, C / params[2]);

	// Maximum multiple of cell vectors we need to go to
	const int maxA = (int)floor(getNumPlaneRepetitionsToBoundSphere(A, B, C, cutoff + abs(rDotA)));
	const int maxB = (int)floor(getNumPlaneRepetitionsToBoundSphere(B, A, C, cutoff + abs(rDotB)));
	const int maxC = (int)floor(getNumPlaneRepetitionsToBoundSphere(C, A, B, cutoff + abs(rDotC)));

  const double cutoffSq = cutoff * cutoff;
  size_t numFound = 0;
  double aSq, bSq, testDistSq;
  double distA = -maxA * params[0] + rDotA, distB, distC;
	for(int a = -maxA; a <= maxA; ++a)
	{
    aSq = distA * distA; /* aSq = a * params[0] + rDotA; aSq *= aSq; */
    distB = -maxB * params[1] + rDotB;
		for(int b = -maxB; b <= maxB; ++b)
		{
      bSq = distB * distB; /* bSq = b * params[1] + rDotB; bSq *= bSq;*/
      distC = -maxC * params[2] + rDotC;
			for(int c = -maxC; c <= maxC; ++c)
			{
        testDistSq = distC * distC; /* testDistSq = c * params[2] + rDotC; testDistSq *= testDistSq; */
        testDistSq += aSq + bSq;

				if(testDistSq < cutoffSq)
				{
          outValues.push_back(sqrt(testDistSq));
          if(++numFound >= maxValues)
            return false;
				}
        distC += params[2];
      }
      distB += params[1];
		}
    distA += params[0];
	}

  // Completed successfully
  return true;
}

bool UniversalCrystalDistanceCalculator::getVecsBetween(
  const ::arma::vec3 & a,
  const ::arma::vec3 & b,
  const double cutoff,
  ::std::vector< ::arma::vec3> & outValues,
  const size_t maxValues) const
{
  const UnitCell & cell = *myStructure.getUnitCell();
  const double (&params)[6] = cell.getLatticeParams();

	// Make sure a and b are in the unit cell at the origin
  const ::arma::vec3		dR		= cell.wrapVec(b) - cell.wrapVec(a);

	// Get the lattice vectors
  const ::arma::vec3 A(cell.getAVec());
  const ::arma::vec3 B(cell.getBVec());
  const ::arma::vec3 C(cell.getCVec());

  const double safeCutoff = cutoff + sqrt(::arma::dot(dR, dR));

  const double rDotA = ::arma::dot(dR, A / params[0]);
  const double rDotB = ::arma::dot(dR, B / params[1]);
  const double rDotC = ::arma::dot(dR, C / params[2]);

	// Maximum multiple of cell vectors we need to go to
	const int maxA = (int)floor(getNumPlaneRepetitionsToBoundSphere(A, B, C, cutoff + abs(rDotA)));
	const int maxB = (int)floor(getNumPlaneRepetitionsToBoundSphere(B, A, C, cutoff + abs(rDotB)));
	const int maxC = (int)floor(getNumPlaneRepetitionsToBoundSphere(C, A, B, cutoff + abs(rDotC)));

  const double cutoffSq = cutoff * cutoff;
  ::arma::vec3 outVec;
  size_t numFound = 0;
  double aSq, bSq, testDistSq;
  double distA = -maxA * params[0] + rDotA, distB, distC;
	for(int a = -maxA; a <= maxA; ++a)
	{
    aSq = distA * distA; /* aSq = a * params[0] + rDotA; aSq *= aSq; */
    distB = -maxB * params[1] + rDotB;
		for(int b = -maxB; b <= maxB; ++b)
		{
      bSq = distB * distB; /* bSq = b * params[1] + rDotB; bSq *= bSq;*/
      distC = -maxC * params[2] + rDotC;
			for(int c = -maxC; c <= maxC; ++c)
			{
        testDistSq = distC * distC; /* testDistSq = c * params[2] + rDotC; testDistSq *= testDistSq; */
        testDistSq += aSq + bSq;

				if(testDistSq < cutoffSq)
				{
          outVec(0) = distA;
          outVec(1) = distB;
          outVec(2) = distC;
          outValues.push_back(outVec);
          if(++numFound >= maxValues)
            return false;
				}
        distC += params[2];
      }
      distB += params[1];
		}
    distA += params[0];
	}

  // Completed successfully
  return true;
}

bool UniversalCrystalDistanceCalculator::isValid() const
{
  return myStructure.getUnitCell() != NULL;
}

double UniversalCrystalDistanceCalculator::getNumPlaneRepetitionsToBoundSphere(
  const ::arma::vec3 & boundDir,
  const ::arma::vec3 & planeVecA,
  const ::arma::vec3 & planeVecB,
	const double radius) const
{
	// The vector normal to the plane
  const ::arma::vec3 normal = ::arma::cross(planeVecA, planeVecB);
  const double unitCellVolume = myStructure.getUnitCell()->getVolume();

  return radius / unitCellVolume * ::std::sqrt(::arma::dot(normal, normal));
}

}
}
