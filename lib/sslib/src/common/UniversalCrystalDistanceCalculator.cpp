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

  ::arma::vec3 dFrac, dRImg, minDR = dR;
  const ::arma::mat33 & orthoMtx = cell.getOrthoMtx();
	double modDRSq;
	for(int a = -maxA; a <= maxA; ++a)
	{
    dFrac[0] = a;
		for(int b = -maxB; b <= maxB; ++b)
		{
		  dFrac[1] = b;
			for(int c = -maxC; c <= maxC; ++c)
			{
				dFrac[2] = c;

        dRImg = orthoMtx * dFrac + dR;
				
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
    ::std::vector<double> & outDistances,
    const size_t maxDistances) const
{
  const UnitCell & cell = *myStructure.getUnitCell();

	// Make sure cart1 and 2 are in the unit cell at the origin
  const ::arma::vec3 		aUnit	= cell.wrapVec(a);
  const ::arma::vec3 		bUnit	= cell.wrapVec(b);
  const ::arma::vec3		dR		= bUnit - aUnit;

	// Get the lattice vectors
  const ::arma::vec3 A(cell.getAVec());
  const ::arma::vec3 B(cell.getBVec());
  const ::arma::vec3 C(cell.getCVec());

	// Maximum multiple of cell vectors we need to go to
	const int maxA = (int)ceil(getNumPlaneRepetitionsToBoundSphere(A, B, C, cutoff));
	const int maxB = (int)ceil(getNumPlaneRepetitionsToBoundSphere(B, A, C, cutoff));
	const int maxC = (int)ceil(getNumPlaneRepetitionsToBoundSphere(C, A, B, cutoff));

  const double cutoffSq = cutoff * cutoff;
  ::arma::vec3 dFrac, dRImg;
  const ::arma::mat33 & orthoMtx = cell.getOrthoMtx();
  double dRDistSq;
  size_t numDistances = 0;
	for(int a = -maxA; a <= maxA; ++a)
	{
		dFrac[0] = a;
		for(int b = -maxB; b <= maxB; ++b)
		{
		  dFrac[1] = b;
			for(int c = -maxC; c <= maxC; ++c)
			{
				dFrac[2] = c;

        dRImg = orthoMtx * dFrac + dR;
        dRDistSq = ::arma::dot(dRImg, dRImg);

				if(dRDistSq < cutoffSq)
				{
          outDistances.push_back(sqrt(dRDistSq));
          if(++numDistances >= maxDistances)
            return false;
				}
			}
		}
	}

  // Completed successfully
  return true;
}

bool UniversalCrystalDistanceCalculator::getVecsBetween(
  const ::arma::vec3 & a,
  const ::arma::vec3 & b,
  const double cutoff,
  ::std::vector< ::arma::vec3> & outVectors,
  const size_t maxVectors) const
{
  const UnitCell & cell = *myStructure.getUnitCell();

	// Make sure cart1 and 2 are in the unit cell at the origin
  const ::arma::vec3 		aUnit	= cell.wrapVec(a);
  const ::arma::vec3 		bUnit	= cell.wrapVec(b);
  const ::arma::vec3		dR		= bUnit - aUnit;

	// Get the lattice vectors
  const ::arma::vec3 A(cell.getAVec());
  const ::arma::vec3 B(cell.getBVec());
  const ::arma::vec3 C(cell.getCVec());

	// Maximum multiple of cell vectors we need to go to
	int maxA = (int)ceil(getNumPlaneRepetitionsToBoundSphere(A, B, C, cutoff));
	int maxB = (int)ceil(getNumPlaneRepetitionsToBoundSphere(B, A, C, cutoff));
	int maxC = (int)ceil(getNumPlaneRepetitionsToBoundSphere(C, A, B, cutoff));

	const double cutoffSq = cutoff * cutoff;
  ::arma::vec3 dFrac, dRImg;
  const ::arma::mat33 & orthoMtx = cell.getOrthoMtx();
  size_t numVectors = 0;
	for(int a = -maxA; a <= maxA; ++a)
	{
    dFrac[0] = a;
		for(int b = -maxB; b <= maxB; ++b)
		{
      dFrac[1] = b;
			for(int c = -maxC; c <= maxC; ++c)
			{
				dFrac[2] = c;

        dRImg = orthoMtx * dFrac + dR;
				
				if(dot(dRImg, dRImg) < cutoffSq)
				{
          outVectors.push_back(dRImg);
          if(++numVectors >= maxVectors)
            return false;
				}
			}
		}
	}
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