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

#include <vector>

#include <boost/circular_buffer.hpp>
#include <boost/noncopyable.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include "potential/IOptimisationController.h"
#include "utility/IBufferedComparator.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////
namespace utility {
class IStructureComparator;
}

namespace potential {

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

  struct LocationData
  {
    int step;
    utility::IBufferedComparator::ComparisonDataHandle comparisonData;
    double enthalpy;
  };

  class OptimisationPath
  {
    typedef ::boost::circular_buffer<LocationData> PathBuffer;
    PathBuffer myPath;
  public:
    typedef PathBuffer::const_reverse_iterator const_reverse_iterator;

    OptimisationPath(const size_t maxLength);
    void addToPath(
      const int step,
      const common::Structure & structure,
      const OptimisationData & optimisationData,
      utility::IBufferedComparator & comparator
    );

    bool empty() const;

    const_reverse_iterator rbegin() const;
    const_reverse_iterator rend() const;
    const LocationData & back() const;
  };
  typedef UniquePtr<OptimisationPath>::Type OptimisationPathPtr;

  class LandscapeMinimum
  {
  public:
    LandscapeMinimum(OptimisationPathPtr path, const double basinSize);
    void addApproachPath(OptimisationPathPtr path);
    double calculateDistanceTo(
      const LocationData & point,
      utility::IBufferedComparator & comparator
    ) const;
    bool isWithinBasin(
      const LocationData & point,
      utility::IBufferedComparator & comparator
    ) const;
    bool liesOnApproachPath(
      const LocationData & point,
      utility::IBufferedComparator & comparator,
      const double distanceTolerance,
      const double enthalpyTolerance
    ) const;
    const LocationData & minimum() const;
  private:
    ::boost::ptr_vector<OptimisationPath> myApproachPaths;
    size_t myLowestEnthalpyPath;
    double myBasinSize;
  };

  struct StopInfo
  {
    size_t currentStep;
    size_t stopStep;
    void reset()
    {
      currentStep = 0;
      stopStep = 0;
    }
  };

  void terminatePath();

  int myRecordingStartStep;
  int myMinConvergenceSteps;
  ComparatorPtr myComparator;
  ::boost::shared_ptr<utility::IBufferedComparator> myBufferedComparator;
  OptimisationPathPtr myCurrentPath;
  ::std::vector<LandscapeMinimum> myLandscapeMinima;
  StopInfo myStopInfo;
  const bool myTestingMode;
};


}
}

#endif /* LANDSCAPE_EXPLORER_H */
