/*
 * SortedDistanceComparator.cpp
 *
 *  Created on: Aug 17, 2011
 *      Author: muhrin
 */

// INCLUDES /////////////////////////////////////

#include "utility/SortedDistanceComparator.h"

#include <memory>

#include <boost/scoped_ptr.hpp>

#include <armadillo>

#include "common/DistanceCalculator.h"
#include "common/Structure.h"
#include "common/UnitCell.h"
#include "utility/GenericBufferedComparator.h"


namespace sstbx {
namespace utility {

const size_t SortedDistanceComparator::MAX_CELL_MULTIPLES   = 10;
const double SortedDistanceComparator::DEFAULT_TOLERANCE    = 1e-3;

SortedDistanceComparator::SortedDistanceComparator(const double tolerance):
myTolerance(tolerance)
{}

double SortedDistanceComparator::compareStructures(
	const sstbx::common::Structure & str1,
	const sstbx::common::Structure & str2) const
{
  ::std::auto_ptr<const SortedDistanceComparator::DataTyp> comp1(generateComparisonData(str1));
  ::std::auto_ptr<const SortedDistanceComparator::DataTyp> comp2(generateComparisonData(str2));

  return compareStructures(str1, *comp1, str2, *comp2);
}

bool SortedDistanceComparator::areSimilar(
	const sstbx::common::Structure & str1,
	const sstbx::common::Structure & str2) const
{
  ::std::auto_ptr<const SortedDistanceComparator::DataTyp> comp1(generateComparisonData(str1));
  ::std::auto_ptr<const SortedDistanceComparator::DataTyp> comp2(generateComparisonData(str2));

  return areSimilar(str1, *comp1, str2, *comp2);
}

double SortedDistanceComparator::compareStructures(
    const common::Structure & str1,
		const SortedDistanceComparisonData & dist1,
    const common::Structure & str2,
		const SortedDistanceComparisonData & dist2) const
{
	using std::vector;

	const size_t maxI = std::min(dist1.sortedDistances.size(), dist2.sortedDistances.size());
  const double maxDist = std::max(dist1.maxDist, dist2.maxDist);

  double delta = 0.0;
	double rms = 0.0;
  double max = 0.0;
  double sum = 0.0;
  double d1, d2;
	for(size_t i = 0; i < maxI; ++i)
	{
    d1 = dist1.sortedDistances[i];
    d2 = dist2.sortedDistances[i];

    if(d1 < maxDist && d2 < maxDist)
    {
      sum = d1 + d2;
      delta = sum > 0 ? std::abs(d1 - d2) / sum : 0;
      max   = std::max(max, delta);
		  rms   += delta * delta;
    }
	}
	rms = sqrt(rms / maxI);

	return max;
}

bool SortedDistanceComparator::areSimilar(
    const common::Structure & str1,
		const SortedDistanceComparisonData & dist1,
    const common::Structure & str2,
		const SortedDistanceComparisonData & dist2) const
{
	return compareStructures(str1, dist1, str2, dist2) < myTolerance;
}

::std::auto_ptr<SortedDistanceComparisonData>
SortedDistanceComparator::generateComparisonData(const sstbx::common::Structure & str) const
{
  const common::DistanceCalculator & distCalc = str.getDistanceCalculator();

  const common::UnitCell * const unitCell = str.getUnitCell();

  double maxDist = 0.0;
  if(unitCell)
  {
    ::arma::vec3 diag = unitCell->getLongestDiagonal();
    double longestDiag = sqrt(::arma::dot(diag, diag));
    maxDist = 1.75 * longestDiag;
  }

	// Copy over the unit cell so we can do distance calculations
	::std::auto_ptr<SortedDistanceComparisonData> data(new SortedDistanceComparisonData(maxDist));

	// Calculate the distances ...
  populateDistanceVector(str, data->sortedDistances, data->maxDist);
	// ... and sort them
	std::sort(data->sortedDistances.begin(), data->sortedDistances.end());

	return data;
}

::boost::shared_ptr<SortedDistanceComparator::BufferedTyp> SortedDistanceComparator::generateBuffered() const
{
  return ::boost::shared_ptr<IBufferedComparator>(
    new GenericBufferedComparator<SortedDistanceComparator>(*this)
  );
}

void SortedDistanceComparator::populateDistanceVector(
	const ::sstbx::common::Structure & str,
	std::vector<double> & distVec,
	const double cutoff) const
{
  ::arma::mat pos;
	str.getAtomPositions(pos);
  const common::DistanceCalculator & distCalc = str.getDistanceCalculator();

  // Have to do all i, j pairs including i = j and ij/ji to get all the possible distances.
  // TODO: Figure out why I have to do the above!!
	std::vector<double> distances;
	const size_t numParticles = pos.n_cols;
	for(size_t i = 0; i < numParticles; ++i)
	{
		for(size_t j = 0; j < numParticles; ++j)
		{
			distances.clear();

			distCalc.getDistsBetween(pos.col(i), pos.col(j), cutoff, distances);

			distVec.insert(distVec.end(), distances.begin(), distances.end());
		}
	}
}

}}
