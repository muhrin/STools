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
#include <spl/common/Types.h>

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

  template <typename GeneratorPtr>
  BuildStructures(const int numToGenerate,
      GeneratorPtr structureGenerator);

  template <typename GeneratorPtr>
  BuildStructures(const float atomsMultiplierGenerate,
      GeneratorPtr structureGenerator);

  // From StartBlock ///
  virtual void
  start();
  // End from StartBlock

  // From PipeBlock //
  virtual void
  in(::spipe::common::StructureData * const data);
  // End from PipeBlock

private:
  typedef ::boost::scoped_ptr< ::spl::build_cell::StructureGenerator> StructureGeneratorPtr;

  ::std::string
  generateStructureName(const size_t structureNum) const;
  ::spl::common::StructurePtr
  generateStructure() const;

  const IStructureGeneratorPtr myStructureGenerator;
  const bool myFixedNumGenerate;
  const int myNumToGenerate;
  const float myAtomsMultiplierGenerate;
  const int myMaxAttempts;
#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::mutex myBuildStructuresMutex;
#endif
};

template <typename GeneratorPtr>
BuildStructures::BuildStructures(const int numToGenerate,
    GeneratorPtr structureGenerator):
    Block("Generate Random structures"), myStructureGenerator(
        structureGenerator), myFixedNumGenerate(true), myNumToGenerate(
        numToGenerate), myAtomsMultiplierGenerate(0.0), myMaxAttempts(
        DEFAULT_MAX_ATTEMPTS)
    {
    }

template <typename GeneratorPtr>
BuildStructures::BuildStructures(const float atomsMultiplierGenerate,
    GeneratorPtr structureGenerator) :
    Block("Generate Random structures"), myStructureGenerator(
        structureGenerator), myFixedNumGenerate(false), myNumToGenerate(0), myAtomsMultiplierGenerate(
        atomsMultiplierGenerate), myMaxAttempts(DEFAULT_MAX_ATTEMPTS)
    {
    }

}
}

#endif /* BUILD_STRUCTURES_H */
