/*
 * KeepWithinXPercent.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef KEEP_WITHIN_X_PERCENT_H
#define KEEP_WITHIN_X_PERCENT_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <map>

#include <boost/noncopyable.hpp>
#ifdef SP_ENABLE_THREAD_AWARE
#  include <boost/thread/mutex.hpp>
#endif

#include <pipelib/pipelib.h>

#include <spl/common/StructureProperties.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace blocks {

class KeepWithinXPercent : public Barrier, ::boost::noncopyable
{
  typedef ::spl::utility::Key<double> StructureProperty;

public:
  explicit
  KeepWithinXPercent(const double percent);
  KeepWithinXPercent(const double percent, const StructureProperty & property);
  KeepWithinXPercent(const double percent, const StructureProperty & property, const bool usePerAtom);


  // From Block /////////////////
  virtual void
  in(spipe::common::StructureData * const data);
  // End from Block /////////////

  // From Barrier /////////////////
  virtual size_t
  release();
  virtual bool
  hasData() const;
  // End from Barrier //////////////

private:
  typedef ::std::multimap< double, StructureDataType *> Structures;

  void
  keep(StructureDataType * const structure, const double energy);
  void
  newLowest(StructureDataType * const structure, const double energy);
  double
  getCutoff() const;

  const double myKeepPercent;
  const StructureProperty myStructureProperty;
  const bool myUsePerAtom;

  Structures myStructures;

#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::mutex myMutex;
#endif
};

}
}

#endif /* KEEP_WITHIN_X_PERCENT_H */
