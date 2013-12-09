/*
 * PointSeparator.h
 *
 *  Created on: Nov 28, 2013
 *      Author: Martin Uhrin
 */

#ifndef POINT_SEPARATOR_H
#define POINT_SEPARATOR_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#include <map>
#include <vector>

#include <armadillo>

#include "spl/common/AtomSpeciesId.h"
#include "spl/utility/Range.h"

namespace spl {
namespace common {
class Atom;
class DistanceCalculator;
class Structure;
}

namespace build_cell {

template <typename Label>
struct SeparationData
{
  typedef ::std::vector< ::arma::vec3> Points;
  typedef ::std::vector<Label> Labels;
  typedef utility::MinMax<Label> LabelPair;
  typedef ::std::map< LabelPair, double> PointSeparations;
  typedef ::std::vector<size_t> FixedPoints;

  static SeparationData<Label>
  fromStructure(const common::Structure & structure);

  SeparationData(const size_t numPoints,
      const common::DistanceCalculator & distanceCalculator);

  const common::DistanceCalculator & distanceCalculator;
  Points points;
  Labels labels;
  PointSeparations separations;
  FixedPoints fixedPoints;
};

SeparationData<common::AtomSpeciesId::Value>
makeSeparationData(const common::Structure & structure);

class PointSeparator
{
public:
  static const int DEFAULT_MAX_ITERATIONS;
  static const double DEFAULT_TOLERANCE;

  PointSeparator();
  PointSeparator(const size_t maxIterations, const double tolerance);

  template <typename Label>
  bool
  separatorPoints(SeparationData<Label> * const sepData) const;
private:
  typedef ::std::vector< bool> FixedList;

  template <typename Label>
  void
  generateMinSepSqs(const SeparationData<Label> & sepData,
      ::arma::mat * const sepSqs) const;
  template <typename Label>
  void
  generateFixedList(const SeparationData<Label> & sepData,
      FixedList * const fixed) const;
  template <typename Label>
  double
  calcMaxOverlapFraction(const SeparationData<Label> & sepData,
      const ::arma::mat & minSepSqs, const FixedList & fixed) const;
  template <typename Label>
  bool
  separatePoints(const ::arma::mat & minSepSqs, const FixedList & fixed,
      SeparationData<Label> * const sepData) const;

  const size_t myMaxIterations;
  const double myTolerance;
};


}
}

#include "spl/build_cell/detail/PointSeparator.h"

#endif /* POINT_SEPARATOR_H */
