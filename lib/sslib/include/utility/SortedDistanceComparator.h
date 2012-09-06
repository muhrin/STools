/*
 * SortedDistanceComparator.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SORTED_DISTANCE_COMPARATOR_H
#define SORTED_DISTANCE_COMPARATOR_H

// INCLUDES /////////////////////////////////////////////
#include <vector>

#include <boost/shared_ptr.hpp>

#include "utility/IStructureComparator.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx {
namespace common {
class Structure;
}
}

namespace sstbx {
namespace utility {

struct SortedDistanceComparisonData
{
	SortedDistanceComparisonData(const double _maxDist):
		maxDist(_maxDist) {}

	::std::vector<double>						            sortedDistances;
  const double                                maxDist;
};

class SortedDistanceComparator : public IStructureComparator
{
public:

	typedef SortedDistanceComparisonData DataTyp;
  typedef IBufferedComparator   BufferedTyp;

	SortedDistanceComparator(double tolerance = DEFAULT_TOLERANCE);

  // From IStructureComparator ////////////////

	virtual double compareStructures(
		const sstbx::common::Structure & str1,
		const sstbx::common::Structure & str2) const;

	virtual bool areSimilar(
		const sstbx::common::Structure & str1,
		const sstbx::common::Structure & str2) const;

  virtual ::boost::shared_ptr<BufferedTyp> generateBuffered() const;

  // End from IStructureComparator /////////////

  // Methods needed to conform to expectations laid out by GenericBufferedComparator ///
	double compareStructures(
    const common::Structure & str1,
		const SortedDistanceComparisonData & dist1,
    const common::Structure & str2,
		const SortedDistanceComparisonData & dist2) const;

	bool areSimilar(
    const common::Structure & str1,
		const SortedDistanceComparisonData & dist1,
    const common::Structure & str2,
		const SortedDistanceComparisonData & dist2) const;

  ::std::auto_ptr<SortedDistanceComparisonData>
    generateComparisonData(const ::sstbx::common::Structure & str) const;
  // End conformation methods //////////////
	

private:

	static const double DEFAULT_TOLERANCE;

	static const size_t MAX_CELL_MULTIPLES;

	void populateDistanceVector(
		const ::sstbx::common::Structure & str,
		::std::vector<double> & distVec,
		const double cutoff) const;

	double myTolerance;

};

}}

#endif /* SORTED_DISTANCE_COMPARATOR_H */
