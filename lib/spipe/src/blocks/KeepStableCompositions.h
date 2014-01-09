/*
 * KeepStableCompositions.h
 *
 *
 *  Created on: Nov 21, 2013
 *      Author: Martin Uhrin
 */

#ifndef KEEP_STABLE_COMPOSITIONS_H
#define KEEP_STABLE_COMPOSITIONS_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <spl/SSLib.h>

#ifdef SPL_WITH_CGAL

#include <set>

#include <boost/scoped_ptr.hpp>
#include <boost/noncopyable.hpp>
#ifdef SP_ENABLE_THREAD_AWARE
#  include <boost/thread/mutex.hpp>
#endif

#include <pipelib/pipelib.h>

#include <spl/analysis/ConvexHullOutputter.h>
#include <spl/analysis/ConvexHullStructures.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace spl {
namespace analysis {
class ConvexHull;
}
namespace common {
class Structure;
}
}

namespace spipe {
namespace blocks {

class KeepStableCompositions : public Barrier, ::boost::noncopyable
{
public:
  KeepStableCompositions();
  KeepStableCompositions(const bool writeHull);

  // From Block /////////////////
  virtual void
  pipelineInitialised();
  virtual void
  in(common::StructureData * const data);
  virtual void
  pipelineFinishing();
  // End from Block /////////////

  // From Barrier /////////////////
  virtual size_t
  release();
  virtual bool
  hasData() const;
  // End from Barrier //////////////

private:
  typedef ::std::set< common::StructureData *> StructureStore;

  const bool myWriteHull;
  ::spl::UniquePtr< ::spl::analysis::ConvexHullOutputter>::Type myOutputter;
  StructureStore myStructureStore;
  mutable ::boost::scoped_ptr< ::spl::analysis::ConvexHullStructures> myConvexHullStructures;

#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::mutex myMutex;
#endif
};

}
}

#endif // SPL_WITH_CGAL
#endif /* KEEP_STABLE_COMPOSITIONS_H */
