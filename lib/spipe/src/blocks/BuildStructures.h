/*
 * BuildStructures.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef BUILD_STRUCTURES_H
#define BUILD_STRUCTURES_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/variant.hpp>
#ifdef SP_ENABLE_THREAD_AWARE
#  include <boost/thread/mutex.hpp>
#endif

#include <spl/build_cell/BuildCellFwd.h>

#include <pipelib/pipelib.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace blocks {

class BuildStructures : public virtual StartBlock,
    public virtual PipeBlock,
    ::boost::noncopyable
{
public:
  typedef ::spl::build_cell::IStructureGeneratorPtr IStructureGeneratorPtr;

  static const int DEFAULT_MAX_ATTEMPTS;

  BuildStructures(const int numToGenerate,
      IStructureGeneratorPtr structureGenerator = IStructureGeneratorPtr());

  BuildStructures(const float atomsMultiplierGenerate,
      IStructureGeneratorPtr structureGenerator = IStructureGeneratorPtr());

  // From StartBlock ///
  virtual void
  start();
  // End from StartBlock

  // From PipeBlock //
  virtual void
  in(::spipe::common::StructureData * const data);
  // End from PipeBlock

private:
  typedef ::boost::scoped_ptr< ::spl::build_cell::IStructureGenerator> StructureGeneratorPtr;

  ::spl::build_cell::IStructureGenerator *
  getStructureGenerator();
  ::std::string
  generateStructureName(const size_t structureNum) const;

  const IStructureGeneratorPtr myStructureGenerator;
  const bool myFixedNumGenerate;
  const int myNumToGenerate;
  const float myAtomsMultiplierGenerate;
  const int myMaxAttempts;
#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::mutex myBuildStructuresMutex;
#endif
};

}
}

#endif /* BUILD_STRUCTURES_H */
