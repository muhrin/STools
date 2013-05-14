/*
 * Landscape.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef LANDSCAPE_H
#define LANDSCAPE_H

// INCLUDES /////////////////////////////////////////////
#include "SSLib.h"

#include <vector>

#include <boost/circular_buffer.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/shared_ptr.hpp>

#include "math/RunningStats.h"
#include "utility/IBufferedComparator.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////
namespace common {
class Structure;
}
namespace utility {
class IStructureComparator;
}

namespace potential {

class Landscape
{
public:
  typedef UniquePtr<utility::IStructureComparator>::Type ComparatorPtr;

  static const size_t DEFAULT_MAX_PATH_LENGTH;
  static const double DEFAULT_ENTHALPY_MATCH_TOLERANCE;
  static const double DEFAULT_CATCHMENT_SIZE_FACTOR;
  static const double DEFAULT_CATCHMENT_ENTHALPY_DELTA_FACTOR;

  struct Point
  {
    utility::IBufferedComparator::ComparisonDataHandle comparisonData;
    double enthalpy;
  };

  class Path : ::boost::noncopyable
  {
    typedef ::boost::circular_buffer<Point> PathBuffer;
    PathBuffer myPath;
  public:
    typedef PathBuffer::const_reverse_iterator const_reverse_iterator;

    Path(utility::IBufferedComparator & comparator, const size_t maxLength);

    void pushBack(
      const common::Structure & structure,
      const double enthalpy
    );
    bool empty() const;

    const_reverse_iterator rbegin() const;
    const_reverse_iterator rend() const;
    const Point & back() const;
  private:
    utility::IBufferedComparator & myComparator;
  };

  class Minimum
  {
  public:

    struct PathQueryResult
    {
      enum Value {NOT_FOUND, ON_APPROACH, AT_MINIMUM};
    };

    Minimum(
      const common::Structure & structure,
      const double enthalpy,
      utility::IBufferedComparator & comparator,
      const double initialCatchmentSize
    );
    Minimum(
      const Path & approachPath,
      utility::IBufferedComparator & comparator,
      const double initialCatchmentSize
    );
    Minimum(const Minimum & toCopy);
    Minimum & operator =(const Minimum & rhs);

    void addApproachPath(const Path & path);
    double distanceTo(const Point & point) const;
    PathQueryResult::Value pathQuery(
      const Path & testPath,
      const double distanceTolerance,
      const double enthalpyTolerance
    );
    bool isWithinCatchment(const Point & testPoint) const;
    bool isAtMinimum(
      const Point & testPoint,
      const double distanceTolerance,
      const double enthalpyTolerance
    ) const;
    double getCatchmentSize() const;
    const Point & point() const;
  private:
    static const double CATCHMENT_VARIATION_FACTOR;

    void updateCatchment(const Path & approachPath, const bool isApproaching);

    utility::IBufferedComparator & myComparator;
    Point myPoint;
    ::std::vector<const Path *> myApproachPaths;
    math::RunningStats myCatchmentStats;
  };

  Landscape(ComparatorPtr comparator);

private:
  typedef ::std::vector<Minimum> Minima;
  typedef ::boost::ptr_vector<Path> Paths;

  ComparatorPtr myComparator;
  ::boost::shared_ptr<utility::IBufferedComparator> myBufferedComparator;
  Minima myMinima;
  Paths myPaths;
  double myDistanceMatchTolerance;
  double myEnthalpyMatchTolerance;
  double myDefaultCatchmentSize;
  double myDefaultCatchmentEnthalpyDelta;

  void initLandscapeMetrics();

  bool pointsMatch(const Point & a, const Point & b) const;

public:
  typedef Minima::iterator MinimaIterator;
  typedef Paths::iterator PathsIterator;
  typedef Minimum::PathQueryResult PathQueryResult;
  typedef ::std::pair<MinimaIterator, PathQueryResult::Value> PathQueryReturn;

  PathsIterator beginPaths();
  PathsIterator endPaths();
  PathsIterator newPath();
  PathsIterator newPath(const common::Structure & structure, const double enthalpy);
  PathsIterator erasePath(Paths::iterator path);

  MinimaIterator beginMinima();
  MinimaIterator endMinima();
  MinimaIterator newMinimum(const common::Structure & structure, const double enthalpy);
  MinimaIterator newMinimum(Paths::iterator path);

  bool isAtMinimum(const MinimaIterator & minimum, const common::Structure & structure, const double enthalpy) const;

  PathQueryReturn pathQuery(const Paths::iterator & approachPath);
};


}
}

#endif /* LANDSCAPE_H */
