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

#include "common/AbstractFmidCell.h"
#include "utility/IStructureComparator.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx
{
namespace common
{
class Structure;
}
}

namespace sstbx
{
namespace utility
{

struct SortedDistanceComparisonData
{
	SortedDistanceComparisonData(
    const ::sstbx::common::AbstractFmidCell & _cell ,
    const double _maxDist):
		cell(_cell), maxDist(_maxDist) {}

	const ::sstbx::common::AbstractFmidCell	cell;
	::std::vector<double>						sortedDistances;
  const double                    maxDist;
};

class SortedDistanceComparator : public IStructureComparator
{
public:

	typedef SortedDistanceComparisonData DataTyp;
  /*typedef GenericBufferedComparator<
    SortedDistanceComparator,
    SortedDistanceComparator::DataTyp> BufferedTyp*/
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
		const SortedDistanceComparisonData & dist1,
		const SortedDistanceComparisonData & dist2) const;

	bool areSimilar(
		const SortedDistanceComparisonData & dist1,
		const SortedDistanceComparisonData & dist2) const;
  // End conformation methods //////////////

	virtual const DataTyp * generateComparisonData(const ::sstbx::common::Structure & str) const;

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
