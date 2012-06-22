/*
 * RandomCellDescription.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef RANDOM_CELL_DESCRIPTION_H
#define RANDOM_CELL_DESCRIPTION_H

// INCLUDES //////////////////////////////////////////////

#include <boost/optional.hpp>

// DEFINES ///////////////////////////////////////////////

// FORWARD DECLARES //////////////////////////////////////
namespace sstbx { namespace build_cell {
	template <typename FloatType>
	class RandomCellGenerator;
}}

namespace sstbx { namespace build_cell {

template <typename FloatType = double>
class RandomCellDescription
{
public:

	friend class RandomCellGenerator<FloatType>;

	void setLatticeParams(
		const FloatType a, const FloatType b, const FloatType c,
		const FloatType alpha, const FloatType beta, const FloatType gamma);

	void setLatticeParams(const FloatType (&latticeParams)[6]);

  ::boost::optional<FloatType> myVolume;
  ::boost::optional<FloatType> myVolDelta;
  ::boost::optional<FloatType> myMaxLengthRatio;
  ::boost::optional<FloatType> myMinAngle;
  ::boost::optional<FloatType> myMaxAngle;
  /** Lattice lengths */
  ::boost::optional<FloatType> myA;
  ::boost::optional<FloatType> myB;
  ::boost::optional<FloatType> myC;
  /** Lattice angles */
  ::boost::optional<FloatType> myAlpha;
  ::boost::optional<FloatType> myBeta;
  ::boost::optional<FloatType> myGamma;

};

}}

// IMPLEMENTATION ///////////////////////////////////////////

namespace sstbx { namespace build_cell {

template <typename FloatType>
void RandomCellDescription<FloatType>::setLatticeParams(
	const FloatType a, const FloatType b, const FloatType c,
	const FloatType alpha, const FloatType beta, const FloatType gamma)
{
  myA.reset(a);
  myB.reset(b);
  myC.reset(b);

  myAlpha.reset(alpha);
  myBeta.reset(beta);
  myGamma.reset(gamma);
}

template <typename FloatType>
void RandomCellDescription<FloatType>::setLatticeParams(const FloatType (&latticeParams)[6])
{
  myA.reset(latticeParams[0]);
	myB.reset(latticeParams[1]);
	myC.reset(latticeParams[2]);
	myAlpha.reset(latticeParams[3]);
	myBeta.reset(latticeParams[4]);
	myGamma.reset(latticeParams[5]);
}


}}

#endif /* RANDOM_CELL_DESCRIPTION_H */
