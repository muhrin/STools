/*
 * LandscapeExplorer.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/LandscapeExplorer.h"

#include <limits>

#include <boost/foreach.hpp>

#include "utility/IStructureComparator.h"
#include "utility/StableComparison.h"

// NAMESPACES ////////////////////////////////
namespace sstbx {
namespace potential {

namespace stable = utility::stable;

// CONSTANTS ////////////////////////////////////////////////
const size_t LandscapeExplorer::MAX_PATH_LENGTH = 50;
const double LandscapeExplorer::DEFAULT_DISTANCE_TOLERANCE = 3e-4;
const double LandscapeExplorer::DEFAULT_BASIN_SIZE = 50 * DEFAULT_DISTANCE_TOLERANCE;
const double LandscapeExplorer::DEFAULT_ENTHALPY_TOLERANCE = 1e-2;

// OptimisationPath ////////////////

LandscapeExplorer::OptimisationPath::OptimisationPath(const size_t maxLength):
myPath(maxLength)
{}

void LandscapeExplorer::OptimisationPath::addToPath(
  const int step,
  const common::Structure & structure,
  const OptimisationData & optimisationData,
  utility::IBufferedComparator & comparator
)
{
  LocationData loc;
  loc.step = step;
  loc.comparisonData = comparator.generateComparisonData(structure);
  loc.enthalpy = *optimisationData.enthalpy;
  myPath.push_back(loc);
}

bool LandscapeExplorer::OptimisationPath::empty() const
{
  return myPath.empty();
}

const LandscapeExplorer::LocationData &
LandscapeExplorer::OptimisationPath::back() const
{
  return myPath.back();
}

// LandscapeMinimum ///////////////////

LandscapeExplorer::LandscapeMinimum::LandscapeMinimum(
  OptimisationPathPtr path,
  const double basinSize
):
myBasinSize(basinSize)
{
  myApproachPaths.push_back(path.release());
  myLowestEnthalpyPath = 0;
}

void LandscapeExplorer::LandscapeMinimum::addApproachPath(OptimisationPathPtr path)
{
  // Check if this is lower in enthalpy than we've seen before
  const OptimisationPath & newPath =
    *myApproachPaths.insert(myApproachPaths.end(), path.release());
  if(minimum().enthalpy > newPath.back().enthalpy)
    myLowestEnthalpyPath = myApproachPaths.size() - 1;
}

double LandscapeExplorer::LandscapeMinimum::calculateDistanceTo(
  const LocationData & point,
  utility::IBufferedComparator & comparator) const
{
  // Calculate the distance from my minimum to the point in hyperspace
  return comparator.compareStructures(
    minimum().comparisonData,
    point.comparisonData
  );
}

bool LandscapeExplorer::LandscapeMinimum::isWithinBasin(
  const LocationData & point,
  utility::IBufferedComparator & comparator
) const
{
  return calculateDistanceTo(point, comparator) <= myBasinSize;
}

bool LandscapeExplorer::LandscapeMinimum::liesOnApproachPath(
  const LocationData & point,
  utility::IBufferedComparator & comparator,
  const double distanceTolerance,
  const double enthalpyTolerance
) const
{
  typedef OptimisationPath::const_reverse_iterator PathIterator;

  // Check if the point is within the enthalpy and distance tolernace of
  // any point on any approach path
  BOOST_FOREACH(const OptimisationPath & path, myApproachPaths)
  {
    for(PathIterator it = path.rbegin(), end = path.rend();
      it != end; ++it)
    {
      if(stable::eq(it->enthalpy, point.enthalpy, enthalpyTolerance))
      {
        const double distance = comparator.compareStructures(it->comparisonData, point.comparisonData);
        if(stable::eq(distance, 0.0, distanceTolerance))
          return true;
      }
    }
  }
  return false;
}

LandscapeExplorer::OptimisationPath::const_reverse_iterator
LandscapeExplorer::OptimisationPath::rbegin() const
{
  return myPath.rbegin();
}

LandscapeExplorer::OptimisationPath::const_reverse_iterator
LandscapeExplorer::OptimisationPath::rend() const
{
  return myPath.rend();
}

const LandscapeExplorer::LocationData &
LandscapeExplorer::LandscapeMinimum::minimum() const
{
  return myApproachPaths[myLowestEnthalpyPath].back();
}

// LandscapeExplorer ///////////////

LandscapeExplorer::LandscapeExplorer(ComparatorPtr comparator):
myComparator(comparator),
myRecordingStartStep(0),
myMinConvergenceSteps(::std::numeric_limits<int>::max()),
myTestingMode(false)
{
  myBufferedComparator = myComparator->generateBuffered();
}

LandscapeExplorer::LandscapeExplorer(ComparatorPtr comparator, const bool testingMode):
myComparator(comparator),
myRecordingStartStep(0),
myMinConvergenceSteps(::std::numeric_limits<int>::max()),
myTestingMode(testingMode)
{
  myBufferedComparator = myComparator->generateBuffered();
}

bool LandscapeExplorer::optimisationStarting(common::Structure & structure)
{
  myCurrentPath.reset(new OptimisationPath(MAX_PATH_LENGTH));
  myStopInfo.reset();
  return true; // Continue
}

bool LandscapeExplorer::stepFinished(
  const int step,
  common::Structure & structure,
  const OptimisationData & optimisationData
)
{
  myStopInfo.currentStep = step;

  // Don't start recording until we're closer to the minimum, otherwise we're wasting time
  // recording a boring part of the path
  if(step < myRecordingStartStep)
    return true;

  // If we would have stopped then don't bother storing any more of the path
  if(myStopInfo.stopStep != 0)
    return true;

  myCurrentPath->addToPath(step, structure, optimisationData, *myBufferedComparator);

  BOOST_FOREACH(const LandscapeMinimum & minimum, myLandscapeMinima)
  {
    if(stable::eq(myCurrentPath->back().enthalpy, minimum.minimum().enthalpy, DEFAULT_ENTHALPY_TOLERANCE) &&
      minimum.isWithinBasin(myCurrentPath->back(), *myBufferedComparator))
    {
      if(minimum.liesOnApproachPath(
        myCurrentPath->back(),
        *myBufferedComparator,
        DEFAULT_DISTANCE_TOLERANCE,
        DEFAULT_ENTHALPY_TOLERANCE
      ))
      {
        if(myStopInfo.stopStep == 0)
        {
          myStopInfo.stopStep = step;
          if(!myTestingMode)
          {
            terminatePath();
            return false;
          }
        }
      }
    }
  }

  return true;
}

void LandscapeExplorer::optimisationFinished(
  const OptimisationOutcome & outcome,
  common::Structure & structure,
  const OptimisationData & optimisationData
)
{
  if(outcome.isSuccess() && myStopInfo.stopStep == 0)
  {
    if(myCurrentPath->empty())
      myMinConvergenceSteps = myStopInfo.currentStep;
    else
    {
      myMinConvergenceSteps = ::std::min(myMinConvergenceSteps, myCurrentPath->back().step);
      myLandscapeMinima.push_back(LandscapeMinimum(myCurrentPath, DEFAULT_BASIN_SIZE));
    }
    myRecordingStartStep =
      myMinConvergenceSteps > MAX_PATH_LENGTH ? myMinConvergenceSteps - MAX_PATH_LENGTH : 0;
  }
  else
    terminatePath();
}

void LandscapeExplorer::terminatePath()
{
  myCurrentPath.reset();
}

}
}

