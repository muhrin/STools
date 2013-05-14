/*
 * LandscapeExplorer.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef LANDSCAPE_EXPLORER_H
#define LANDSCAPE_EXPLORER_H

// INCLUDES /////////////////////////////////////////////
#include "SSLib.h"

#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>

#include "potential/IOptimisationController.h"
#include "potential/Landscape.h"
#include "utility/IBufferedComparator.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////
namespace utility {
class IStructureComparator;
}
namespace potential {
namespace detail {
class ExplorerTester;
}


class LandscapeExplorer : public IOptimisationController
{
public:
  typedef UniquePtr<utility::IStructureComparator>::Type ComparatorPtr;

  LandscapeExplorer(ComparatorPtr comparator);
  LandscapeExplorer(ComparatorPtr comparator, const bool testingMode);

  bool optimisationStarting(common::Structure & structure);
  bool stepFinished(
    const int step,
    common::Structure & structure,
    const OptimisationData & optimisationData
  );
  void optimisationFinished(
    const OptimisationOutcome & outcome,
    common::Structure & structure,
    const OptimisationData & optimisationData
  );

private:
  static const size_t MAX_PATH_LENGTH;
  static const double DEFAULT_DISTANCE_TOLERANCE;
  static const double DEFAULT_ENTHALPY_TOLERANCE;
  static const double DEFAULT_BASIN_SIZE;

  struct StopInfo
  {
    size_t currentStep;
    size_t stopStep;
    Landscape::PathQueryReturn queryResult;
    void reset()
    {
      currentStep = 0;
      stopStep = 0;
    }
  };

  struct TerminateAction
  {
    enum Value
    {
      NONE,
      ADD_APPROACH_PATH,
      ADD_NEW_MINIMUM,
      TERMINATE_PATH
    };
  };

  void terminatePath();

  Landscape myLandscape;
  int myRecordingStartStep;
  int myMinConvergenceSteps;
  ::boost::optional<Landscape::PathsIterator> myCurrentPath;
  StopInfo myStopInfo;
  ::boost::scoped_ptr<detail::ExplorerTester> myTester;

  friend class detail::ExplorerTester;
};

namespace detail {

class ExplorerTester
{
public:

  ExplorerTester(LandscapeExplorer & explorer);
  ~ExplorerTester();

  void optimisationFinished(
    const OptimisationOutcome & outcome,
    const common::Structure & structure,
    const OptimisationData & optimisationData,
    const LandscapeExplorer::TerminateAction::Value explorerAction
  );
private:

  void logMsg(const ::std::string & message) const;
  void writeStats() const;

  const ::std::string myLogFilename;
  int myTotalSteps;
  int myNumFalsePositives;
  math::RunningStats myOptimisationStats;
  math::RunningStats myOverallSavingsStats;
  math::RunningStats mySavingsStats;
  LandscapeExplorer & myExplorer;
};

} // namespace detail
} // namspace potential
} // naespace sstbx

#endif /* LANDSCAPE_EXPLORER_H */
