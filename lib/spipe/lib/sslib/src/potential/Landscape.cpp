/*
 * LandscapeExplorer.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/Landscape.h"

#include <boost/foreach.hpp>

#include "SSLibAssert.h"
#include "common/Structure.h"
#include "utility/IStructureComparator.h"

// NAMESPACES ////////////////////////////////
namespace sstbx {
namespace potential {

namespace stable = utility::stable;

// CONSTANTS ////////////////////////////////////////////////

const size_t Landscape::DEFAULT_MAX_PATH_LENGTH = 50;
const double Landscape::DEFAULT_ENTHALPY_MATCH_TOLERANCE = 0.1;
const double Landscape::DEFAULT_CATCHMENT_SIZE_FACTOR = 100.0;
const double Landscape::DEFAULT_CATCHMENT_ENTHALPY_DELTA_FACTOR = 10.0;

const double Landscape::Minimum::CATCHMENT_VARIATION_FACTOR = 0.1;

Landscape::Path::Path(utility::IBufferedComparator & comparator, const size_t maxLength):
myComparator(comparator),
myPath(maxLength)
{}

void Landscape::Path::pushBack(const common::Structure & structure, const double enthalpy)
{
  Landscape::Point point;
  point.enthalpy = enthalpy;
  point.comparisonData = myComparator.generateComparisonData(structure);
  myPath.push_back(point);
}

bool Landscape::Path::empty() const
{
  return myPath.empty();
}

Landscape::Path::const_reverse_iterator
Landscape::Path::rbegin() const
{
  return myPath.rbegin();
}

Landscape::Path::const_reverse_iterator
Landscape::Path::rend() const
{
  return myPath.rend();
}

const Landscape::Point &
Landscape::Path::back() const
{
  return myPath.back();
}

Landscape::Minimum::Minimum(
  const common::Structure & structure,
  const double enthalpy,
  utility::IBufferedComparator & comparator,
  const double initialCatchmentSize
):
myComparator(comparator)
{
  myPoint.enthalpy = enthalpy;
  myPoint.comparisonData = myComparator.generateComparisonData(structure);
  myCatchmentStats.insert(initialCatchmentSize);
}

Landscape::Minimum::Minimum(const Path & path, utility::IBufferedComparator & comparator, const double initialCatchmentSize):
myComparator(comparator),
myPoint(path.back())
{
  myApproachPaths.push_back(&path);
  myCatchmentStats.insert(initialCatchmentSize);
}

Landscape::Minimum::Minimum(const Minimum & toCopy):
myComparator(toCopy.myComparator),
myPoint(toCopy.myPoint),
myApproachPaths(toCopy.myApproachPaths),
myCatchmentStats(toCopy.myCatchmentStats)
{}

Landscape::Minimum & Landscape::Minimum::operator =(const Minimum & rhs)
{
  SSLIB_ASSERT(&myComparator == &rhs.myComparator);

  myPoint = rhs.myPoint;
  myApproachPaths = rhs.myApproachPaths;
  myCatchmentStats = rhs.myCatchmentStats;
  return *this;
}

void Landscape::Minimum::addApproachPath(const Landscape::Path & path)
{
  myApproachPaths.push_back(&path);
  // We may have found a slightly lower enthalpy point so update
  if(path.back().enthalpy < myPoint.enthalpy)
    myPoint = path.back();
}

double Landscape::Minimum::distanceTo(const Landscape::Point & point) const
{
  // Calculate the distance from my minimum to the point in hyperspace
  return myComparator.compareStructures(myPoint.comparisonData, point.comparisonData);
}

Landscape::Minimum::PathQueryResult::Value
Landscape::Minimum::pathQuery(const Path & testPath, const double distanceTolerance, const double enthalpyTolerance)
{
  typedef Landscape::Path::const_reverse_iterator PathIterator;

  const Point & endOfPath = testPath.back();
  if(!isWithinCatchment(endOfPath))
    return PathQueryResult::NOT_FOUND;

  if(isAtMinimum(endOfPath, distanceTolerance, enthalpyTolerance))
    return PathQueryResult::AT_MINIMUM;

  // Check if the point is within the enthalpy and distance tolernace of
  // any point on any approach path
  double distance;
  BOOST_FOREACH(const Landscape::Path * path, myApproachPaths)
  {
    for(PathIterator it = path->rbegin(), end = path->rend(); it != end; ++it)
    {
      if(stable::eq(it->enthalpy, endOfPath.enthalpy, enthalpyTolerance))
      {
        distance = myComparator.compareStructures(it->comparisonData, endOfPath.comparisonData);
        if(stable::eq(distance, 0.0, distanceTolerance))
        {
          updateCatchment(testPath, true);
          return PathQueryResult::ON_APPROACH;
        }
      }
    }
  }
  updateCatchment(testPath, false);
  return PathQueryResult::NOT_FOUND;
}

bool Landscape::Minimum::isWithinCatchment(const Point & testPoint) const
{
  const double distance = myComparator.compareStructures(myPoint.comparisonData, testPoint.comparisonData);
  return stable::eq(distance, 0.0, getCatchmentSize());
}

bool Landscape::Minimum::isAtMinimum(
  const Point & testPoint,
  const double distanceTolerance,
  const double enthalpyTolerance
) const
{
  if(!stable::eq(myPoint.enthalpy, testPoint.enthalpy, enthalpyTolerance))
    return false;

  const double distance = myComparator.compareStructures(myPoint.comparisonData, testPoint.comparisonData);
  return stable::eq(distance, 0.0, distanceTolerance);
}

double Landscape::Minimum::getCatchmentSize() const
{
  return myCatchmentStats.mean();
}

void Landscape::Minimum::updateCatchment(const Path & approachPath, const bool isApproaching)
{
  const double catchmentVariation = isApproaching ? (1.0 + CATCHMENT_VARIATION_FACTOR) : (1.0 - CATCHMENT_VARIATION_FACTOR);
  myCatchmentStats.insert(myCatchmentStats.mean() * catchmentVariation);
}

const Landscape::Point & Landscape::Minimum::point() const
{
  return myPoint;
}

Landscape::Landscape(ComparatorPtr comparator):
myComparator(comparator)
{
  initLandscapeMetrics();
  myBufferedComparator = myComparator->generateBuffered();
}

void Landscape::initLandscapeMetrics()
{
  myDistanceMatchTolerance = myComparator->getTolerance();
  myEnthalpyMatchTolerance = DEFAULT_ENTHALPY_MATCH_TOLERANCE;
  myDefaultCatchmentSize = DEFAULT_CATCHMENT_SIZE_FACTOR * myDistanceMatchTolerance;
  myDefaultCatchmentEnthalpyDelta = DEFAULT_CATCHMENT_ENTHALPY_DELTA_FACTOR * myEnthalpyMatchTolerance;
}

bool Landscape::pointsMatch(const Point & a, const Point & b) const
{
  return stable::eq(a.enthalpy, b.enthalpy, myEnthalpyMatchTolerance) &&
    stable::eq(myBufferedComparator->compareStructures(a.comparisonData, b.comparisonData), 0.0, myDistanceMatchTolerance);
}

Landscape::PathsIterator Landscape::beginPaths()
{
  return myPaths.begin();
}

Landscape::PathsIterator Landscape::endPaths()
{
  return myPaths.end();
}

Landscape::Paths::iterator Landscape::newPath()
{
  const Paths::iterator it = myPaths.insert(myPaths.end(), new Path(*myBufferedComparator, DEFAULT_MAX_PATH_LENGTH));
  return it;
}

Landscape::Paths::iterator Landscape::newPath(const common::Structure & structure, const double enthalpy)
{
  const Paths::iterator it = newPath();
  it->pushBack(structure, enthalpy);
  return it;
}

Landscape::Paths::iterator Landscape::erasePath(Paths::iterator path)
{
  return myPaths.erase(path);
}

Landscape::MinimaIterator Landscape::beginMinima()
{
  return myMinima.begin();
}

Landscape::MinimaIterator Landscape::endMinima()
{
  return myMinima.end();
}

Landscape::MinimaIterator Landscape::newMinimum(const common::Structure & structure, const double enthalpy)
{
  return myMinima.insert(myMinima.end(), Minimum(structure, enthalpy, *myBufferedComparator, myDefaultCatchmentSize));
}

Landscape::MinimaIterator Landscape::newMinimum(Paths::iterator path)
{
  return myMinima.insert(myMinima.end(), Minimum(*path, *myBufferedComparator, myDefaultCatchmentSize));
}

bool Landscape::isAtMinimum(
  const MinimaIterator & minimum,
  const common::Structure & structure,
  const double enthalpy
) const
{
  Point point;
  point.enthalpy = enthalpy;
  point.comparisonData = myBufferedComparator->generateComparisonData(structure);
  return minimum->isAtMinimum(point, 2.0 * myDistanceMatchTolerance, 2.0 * myEnthalpyMatchTolerance);
}

Landscape::PathQueryReturn
Landscape::pathQuery(const Paths::iterator & approachPath)
{
  Minima::iterator it, end;
  Minimum::PathQueryResult::Value pathQueryResult;
  for(it = myMinima.begin(), end = myMinima.end(); it != end; ++it)
  {
    pathQueryResult = it->pathQuery(*approachPath, myDistanceMatchTolerance, myEnthalpyMatchTolerance);
    if(pathQueryResult != PathQueryResult::NOT_FOUND)
      return PathQueryReturn(it, pathQueryResult);
  }
  return PathQueryReturn(it, PathQueryResult::NOT_FOUND);
}


}
}


