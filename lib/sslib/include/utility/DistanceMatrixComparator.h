/*
 * DistanceMatrixComparator.h
 *
 *
 *  Created on: Aug 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef DISTANCE_MATRIX_COMPARATOR_H
#define DISTANCE_MATRIX_COMPARATOR_H

// INCLUDES /////////////////////////////////////////////
#include <vector>

#include <boost/shared_ptr.hpp>

#include <armadillo>

#include "common/AbstractFmidCell.h"
#include "utility/IStructureComparator.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx {
namespace common {
class Structure;
}
}

namespace sstbx {
namespace utility {

struct DistanceMatrixComparisonData
{
  ::arma::mat   distancesMtx;
};

class DistanceMatrixComparator : public IStructureComparator
{
public:

  typedef DistanceMatrixComparisonData DataTyp;

  // From IStructureComparator ////////////////

	virtual double compareStructures(
		const sstbx::common::Structure & str1,
		const sstbx::common::Structure & str2) const;

	virtual bool areSimilar(
		const sstbx::common::Structure & str1,
		const sstbx::common::Structure & str2) const;

  virtual ::boost::shared_ptr<IBufferedComparator> generateBuffered() const;

  // End from IStructureComparator /////////////

  // Methods needed to conform to expectations laid out by GenericBufferedComparator ///
	double compareStructures(
		const DistanceMatrixComparisonData & dist1,
		const DistanceMatrixComparisonData & dist2) const;

	bool areSimilar(
		const DistanceMatrixComparisonData & dist1,
		const DistanceMatrixComparisonData & dist2) const;
  // End conformation methods //////////////

	virtual const DataTyp * generateComparisonData(const ::sstbx::common::Structure & str) const;

private:


};

}
}

#endif /* DISTANCE_MATRIX_COMPARATOR_H */
