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

// From SSTbx
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

class RandomStructure : public SpStartBlock, ::boost::noncopyable
{
public:

  typedef ::boost::optional<unsigned int> OptionalUInt;
  typedef ::boost::shared_ptr<const ::sstbx::build_cell::StructureDescription> StructureDescPtr;

	RandomStructure(
		const ::sstbx::build_cell::IStructureGenerator & structureGenerator,
    const OptionalUInt numToGenerate = OptionalUInt(),
    const StructureDescPtr & structureDescription = StructureDescPtr());


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

  virtual void initDescriptions();

  double setRadii() const;

	const OptionalUInt myNumToGenerate;

  /** Should the block use the structure description found in shared data */
  const bool  myUseSharedDataStructureDesc;

  StructureDescPtr       myStructureDescription;

	const ::sstbx::build_cell::IStructureGenerator &	myStructureGenerator;

};


}
}

#endif /* RANDOM_STRUCTURE_H */
