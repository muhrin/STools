/*
 * LandscapeExplorer.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/LandscapeExplorer.h"

#include <limits>
#include <fstream>
#include <sstream>

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

LandscapeExplorer::LandscapeExplorer(ComparatorPtr comparator):
myLandscape(comparator),
myRecordingStartStep(0),
myMinConvergenceSteps(::std::numeric_limits<int>::max())
{}

LandscapeExplorer::LandscapeExplorer(ComparatorPtr comparator, const bool testingMode):
myLandscape(comparator),
myRecordingStartStep(0),
myMinConvergenceSteps(::std::numeric_limits<int>::max())
{
  if(testingMode)
    myTester.reset(new detail::ExplorerTester(*this));
}

bool LandscapeExplorer::optimisationStarting(common::Structure & structure)
{
  myCurrentPath.reset(myLandscape.newPath());
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

  (*myCurrentPath)->pushBack(structure, *optimisationData.enthalpy);

  Landscape::PathQueryReturn result = myLandscape.pathQuery(*myCurrentPath);
  if(myStopInfo.stopStep == 0 && result.second != Landscape::Minimum::PathQueryResult::NOT_FOUND)
  {
    myStopInfo.stopStep = step;
    myStopInfo.queryResult = result;

    // In testing mode don't terminate the path
    if(!myTester.get())
    {
      terminatePath();
      return false;
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
  TerminateAction::Value action = TerminateAction::NONE;

  if(outcome.isSuccess())
  {
    // Optimise the recording start point
    if((*myCurrentPath)->empty())
      myMinConvergenceSteps = myStopInfo.currentStep;
    else
      myMinConvergenceSteps = ::std::min(myMinConvergenceSteps, static_cast<int>(myStopInfo.currentStep));
    myRecordingStartStep = myMinConvergenceSteps > MAX_PATH_LENGTH ? myMinConvergenceSteps - MAX_PATH_LENGTH : 0;

    // Have we stopped the optimisation?
    if(myStopInfo.stopStep != 0)
    {
      if(myStopInfo.queryResult.second == Landscape::PathQueryResult::AT_MINIMUM)
      {
        if(!(*myCurrentPath)->empty())
          action = TerminateAction::ADD_APPROACH_PATH;
      }
      else if(myStopInfo.queryResult.second == Landscape::PathQueryResult::ON_APPROACH)
        action = TerminateAction::TERMINATE_PATH;
    }
    else // No: we haven't seen this minimum before
      action = TerminateAction::ADD_NEW_MINIMUM;
  }
  else
    action = TerminateAction::TERMINATE_PATH;


  // Update the tester
  if(myTester.get())
    myTester->optimisationFinished(outcome, structure, optimisationData, action);

  if(action == TerminateAction::ADD_APPROACH_PATH)
    myStopInfo.queryResult.first->addApproachPath(**myCurrentPath);
  else if(action == TerminateAction::ADD_NEW_MINIMUM)
  {
    if((*myCurrentPath)->empty())
      myLandscape.newMinimum(structure, *optimisationData.enthalpy);
    else
      myLandscape.newMinimum(*myCurrentPath);
  }
  else if(action == TerminateAction::TERMINATE_PATH)
    terminatePath();

  myCurrentPath.reset();
}

void LandscapeExplorer::terminatePath()
{
  if(myCurrentPath)
  {
    myLandscape.erasePath(*myCurrentPath);
    myCurrentPath.reset();
  }
}


namespace detail {

ExplorerTester::ExplorerTester(LandscapeExplorer & explorer):
myExplorer(explorer),
myNumFalsePositives(0),
myTotalSteps(0),
myLogFilename("explorer.log")
{}

ExplorerTester::~ExplorerTester()
{
  writeStats();
}

void ExplorerTester::optimisationFinished(
  const OptimisationOutcome & outcome,
  const common::Structure & structure,
  const OptimisationData & optimisationData,
  const LandscapeExplorer::TerminateAction::Value explorerAction
)
{
  // Record how many steps it took
  myOptimisationStats.insert(myExplorer.myStopInfo.currentStep);

  if(outcome.isSuccess())
  {
    if(myExplorer.myStopInfo.stopStep != 0)
    {
      // Check that the structure ended up where we predicted it would go
      if(!myExplorer.myLandscape.isAtMinimum(myExplorer.myStopInfo.queryResult.first, structure, *optimisationData.enthalpy))
      {
        logMsg("Optimisation flagged to stop didn't go to the predicted minimum.");
        ++myNumFalsePositives;
      }
      else
      {
	// Record how much we saved
        mySavingsStats.insert(myExplorer.myStopInfo.currentStep - myExplorer.myStopInfo.stopStep);
	myOverallSavingsStats.insert(myExplorer.myStopInfo.currentStep - myExplorer.myStopInfo.stopStep);
      }
    }
    else
      myOverallSavingsStats.insert(0);
  }
  else
  {
    if(myExplorer.myStopInfo.stopStep != 0)
    {
      // We identifies this as an optimisation to stop because we knew where it was going
      // but it turned out that the optimisation failed
      logMsg("Optimisation flagged to stop has ended in failure.");
      ++myNumFalsePositives;
    }
  }
}

void ExplorerTester::logMsg(const ::std::string & message) const
{
  ::std::ofstream out(myLogFilename.c_str(), ::std::ios_base::out |  ::std::ios_base::app);
  if(out.is_open())
  {
    out << message << ::std::endl;
    out.close();
  }
}

void ExplorerTester::writeStats() const
{
  ::std::stringstream ss;
  ss << "False positives: " << myNumFalsePositives << ::std::endl;
  ss << "Steps stats " << myOptimisationStats << ::std::endl;
  ss << "Savings stats " << mySavingsStats << ", "
    << mySavingsStats.mean() / myOptimisationStats.mean() * 100.0 << "%" << ::std::endl;
  ss << "Overall savings stats " << myOverallSavingsStats << ", "
    << myOverallSavingsStats.mean() / myOptimisationStats.mean() * 100.0 << "%";
  logMsg(ss.str());
}

} // namespace detail
} // namespace potential
} // namespace sstbx

