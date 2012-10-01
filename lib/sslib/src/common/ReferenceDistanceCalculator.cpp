/*
 * ReferenceDistanceCalculator.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES ///////////////
#include "common/ReferenceDistanceCalculator.h"

#include "common/Structure.h"
#include "common/UnitCell.h"


namespace sstbx {
namespace common {

ReferenceDistanceCalculator::ReferenceDistanceCalculator(const Structure & structure):
DistanceCalculator(structure)
{}

::arma::vec3 ReferenceDistanceCalculator::getVecMinImg(const ::arma::vec3 & a, const ::arma::vec3 & b) const
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

bool ReferenceDistanceCalculator::getDistsBetween(
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

	// Maximum multiple of cell vectors we need to go to
	const int A_max = (int)ceil(getNumPlaneRepetitionsToBoundSphere(A, B, C, safeCutoff));
	const int B_max = (int)ceil(getNumPlaneRepetitionsToBoundSphere(B, A, C, safeCutoff));
	const int C_max = (int)ceil(getNumPlaneRepetitionsToBoundSphere(C, A, B, safeCutoff));

  const double cutoffSq = cutoff * cutoff;
  ::arma::vec3 outVec;
  size_t numFound = 0;
  ::arma::vec3 r;
  double rSq;
	for(int a = -A_max; a <= A_max; ++a)
	{
		for(int b = -B_max; b <= B_max; ++b)
		{
			for(int c = -C_max; c <= C_max; ++c)
			{
        r = a * A + b * B + c * C + dR;
        rSq = ::arma::dot(r, r);

				if(rSq < cutoffSq)
        {
          outValues.push_back(sqrt(rSq));
          if(++numFound >= maxValues)
            return false;
        }
      }
		}
	}
  return true;
}

bool ReferenceDistanceCalculator::getVecsBetween(
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

	// Maximum multiple of cell vectors we need to go to
	const int A_max = (int)ceil(getNumPlaneRepetitionsToBoundSphere(A, B, C, safeCutoff));
	const int B_max = (int)ceil(getNumPlaneRepetitionsToBoundSphere(B, A, C, safeCutoff));
	const int C_max = (int)ceil(getNumPlaneRepetitionsToBoundSphere(C, A, B, safeCutoff));

  const double cutoffSq = cutoff * cutoff;
  ::arma::vec3 outVec;
  size_t numFound = 0;
  ::arma::vec3 r;
	for(int a = -A_max; a <= A_max; ++a)
	{
		for(int b = -B_max; b <= B_max; ++b)
		{
			for(int c = -C_max; c <= C_max; ++c)
			{
        r = a * A + b * B + c * C + dR;

				if(::arma::dot(r, r) < cutoffSq)
        {
          outValues.push_back(r);
          if(++numFound >= maxValues)
            return false;
        }
      }
		}
	}
  return true;
}

bool ReferenceDistanceCalculator::isValid() const
{
  return myStructure.getUnitCell() != NULL;
}

double ReferenceDistanceCalculator::getNumPlaneRepetitionsToBoundSphere(
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
