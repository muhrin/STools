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
namespace sstbx {
namespace build_cell {
	template <typename FloatType>
	class RandomCellGenerator;
}
}

namespace sstbx {
namespace build_cell {

class RandomCellDescription
{
public:

	friend class RandomCellGenerator<double>;

	void setLatticeParams(
		const double a, const double b, const double c,
		const double alpha, const double beta, const double gamma);

	void setLatticeParams(const double (&latticeParams)[6]);

  ::boost::optional<double> myVolume;
  ::boost::optional<double> myVolDelta;
  ::boost::optional<double> myMaxLengthRatio;
  ::boost::optional<double> myMinAngle;
  ::boost::optional<double> myMaxAngle;
  /** Lattice lengths */
  ::boost::optional<double> myA;
  ::boost::optional<double> myB;
  ::boost::optional<double> myC;
  /** Lattice angles */
  ::boost::optional<double> myAlpha;
  ::boost::optional<double> myBeta;
  ::boost::optional<double> myGamma;

};

}
}


#endif /* RANDOM_CELL_DESCRIPTION_H */
