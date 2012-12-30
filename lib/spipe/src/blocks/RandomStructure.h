/*
 * RandomStructure.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef RANDOM_STRUCTURE_H
#define RANDOM_STRUCTURE_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/variant.hpp>

// From SSTbx
#include <build_cell/IStructureGenerator.h>
#include <build_cell/StructureDescription.h>

#include <pipelib/pipelib.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace build_cell {
class IStructureGenerator;
}
}

namespace spipe {
namespace blocks {

class RandomStructure : public virtual SpStartBlock, public virtual SpPipeBlock,
  ::boost::noncopyable
{
public:

  typedef ::boost::shared_ptr<const ::sstbx::build_cell::StructureDescription> StructureDescPtr;

	RandomStructure(
    const unsigned int numToGenerate,
    const StructureDescPtr & structureDescription = StructureDescPtr()
  );

  RandomStructure(
    const float atomsMultiplierGenerate,
    const StructureDescPtr & structureDescription = StructureDescPtr()
  );


  // From Block ////////
  virtual void pipelineStarting();
  // End from Block ////

  // From StartBlock ///
	virtual void start();
  // End from StartBlock

  // From PipeBlock //
	virtual void in(::spipe::common::StructureData & data);
  // End from PipeBlock

private:
  typedef ::boost::scoped_ptr< ::sstbx::build_cell::IStructureGenerator> StructureGeneratorPtr;

  virtual void initDescriptions();

  double setRadii() const;

  /** Should the block use the structure description found in shared data */
  const bool  myUseSharedDataStructureDesc;

  StructureDescPtr       myStructureDescription;

	const StructureGeneratorPtr myStructureGenerator;
  const bool myFixedNumGenerate;
  const unsigned int myNumToGenerate;
  const float myAtomsMultiplierGenerate;
};


}
}

#endif /* RANDOM_STRUCTURE_H */
