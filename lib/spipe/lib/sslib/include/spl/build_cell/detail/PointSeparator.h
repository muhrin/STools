/*
 * PointSeparator.h
 *
 *  Created on: Nov 28, 2013
 *      Author: Martin Uhrin
 */

#ifndef POINT_SEPARATOR_DETAIL_H
#define POINT_SEPARATOR_DETAIL_H

// INCLUDES ////////////
#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>

#include "spl/common/Structure.h"

namespace spl {
namespace build_cell {

template< typename Label>
  SeparationData< Label>::SeparationData(const size_t numPoints,
      const common::DistanceCalculator & calculator) :
      distanceCalculator(calculator), points(numPoints), labels(numPoints)
  {
  }

template< typename Label>
  bool
  PointSeparator::separatorPoints(SeparationData<Label> * const sepData) const
  {
    ::arma::mat minSepSq;
    FixedList fixedList;
    generateFixedList(*sepData, &fixedList);
    generateMinSepSqs(*sepData, &minSepSq);

    return separatePoints(minSepSq, fixedList, sepData);
  }

template< typename Label>
  void
  PointSeparator::generateMinSepSqs(
      const SeparationData< Label> & sepData, ::arma::mat * const sepSqs) const
  {
    typedef SeparationData< Label> SepData;

    const size_t numPoints = sepData.points.size();
    sepSqs->resize(numPoints, numPoints);

    typename SepData::PointSeparations::const_iterator it;
    for(size_t row = 0; row < numPoints - 1; ++row)
    {
      const Label pt1Label = sepData.labels[row];
      for(size_t col = row; col < numPoints; ++col)
      {
        it = sepData.separations.find(
            typename SepData::LabelPair(pt1Label, sepData.labels[row]));
        (*sepSqs)(row, col) =
            it != sepData.separations.end() ? it->second : 0.0;
      }
    }
  }

template< typename Label>
  void
  PointSeparator::generateFixedList(
      const SeparationData< Label> & sepData, FixedList * const fixed) const
  {
    fixed->assign(sepData.points.size(), false);
    BOOST_FOREACH(const size_t idx, sepData.fixedPoints)
      (*fixed)[idx] = true;
  }

template< typename Label>
  double
  PointSeparator::calcMaxOverlapFraction(
      const SeparationData< Label> & sepData, const ::arma::mat & minSepSqs,
      const FixedList & fixed) const
  {
    const size_t numPoints = sepData.points.size();

    double sepSq, maxOverlapSq = 0.0;
    for(size_t row = 0; row < numPoints - 1; ++row)
    {
      const ::arma::vec & posI = sepData.points[row];
      for(size_t col = row + 1; col < numPoints; ++col)
      {
        if(!(fixed[row] && fixed[col])) // Only if they're not both fixed
        {
          const ::arma::vec & posJ = sepData.points[col];

          sepSq = sepData.distanceCalculator.getDistSqMinImg(posI, posJ);

          if(sepSq < minSepSqs(row, col))
            maxOverlapSq = ::std::max(maxOverlapSq,
                minSepSqs(row, col) / sepSq);
        }
      }
    }
    return 1.0 - 1.0 / ::std::sqrt(maxOverlapSq);
  }

template< typename Label>
  bool
  PointSeparator::separatePoints(const ::arma::mat & minSepSqs, const FixedList & fixed,
      SeparationData<Label> * const sepData) const
  {
    using ::std::sqrt;

    const size_t numPoints = sepData->points.size();
    if(numPoints == 0)
      return true;

    double sep, sepSq, sepDiff;
    double maxOverlapFraction;
    double prefactor; // Used to adjust the displacement vector if either atom is fixed
    ::arma::vec3 dr, sepVec;
    bool success = false;

    for(size_t iters = 0; iters < myMaxIterations; ++iters)
    {
      // First loop over calculating separations and checking for overlap
      maxOverlapFraction = calcMaxOverlapFraction(*sepData, minSepSqs, fixed);

      if(maxOverlapFraction < myTolerance)
      {
        success = true;
        break;
      }

      // Now fix-up any overlaps
      for(size_t row = 0; row < numPoints - 1; ++row)
      {
        const ::arma::vec & posI = sepData->points[row];
        for(size_t col = row + 1; col < numPoints; ++col)
        {
          if(!(fixed[row] && fixed[col]))
          {
            const ::arma::vec & posJ = sepData->points[col];
            sepVec = sepData->distanceCalculator.getVecMinImg(posI, posJ);
            sepSq = ::arma::dot(sepVec, sepVec);
            if(sepSq < minSepSqs(row, col))
            {
              if(fixed[row] || fixed[col])
                prefactor = 1.0; // Only one fixed, displace it by the full amount
              else
                prefactor = 0.5; // None fixed, share displacement equally

              if(sepSq != 0.0)
              {
                sep = sqrt(sepSq);
                sepDiff = sqrt(minSepSqs(row, col)) - sep;

                // Generate the displacement vector
                dr = prefactor * sepDiff / sep * sepVec;

                if(!fixed[row])
                  sepData->points[row] -= dr;
                if(!fixed[col])
                  sepData->points[col] += dr;
              }
              else
              {
                // TODO: Perturb the atoms a little to get them moving
              }
            }
          }
        }
      }
    }
    return success;
  }

}
}

#endif /* POINT_SEPARATOR_DETAIL_H */
