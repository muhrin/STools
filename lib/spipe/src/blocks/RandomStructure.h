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

#include <boost/shared_ptr.hpp>

// From SSTbx
#include <build_cell/RandomCellDescription.h>
#include <build_cell/StructureDescription.h>

// From PipelineLib
#include <pipelib/AbstractSimpleStartBlock.h>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
namespace build_cell
{
class ICrystalStructureGenerator;
}
}

namespace spipe
{
namespace blocks {

class RandomStructure :
	public pipelib::AbstractSimpleStartBlock<StructureDataTyp, SharedDataTyp>
{
public:

  typedef ::boost::shared_ptr<const ::sstbx::build_cell::StructureDescription >           StructureDescPtr;
  typedef ::boost::shared_ptr<const ::sstbx::build_cell::RandomCellDescription<double> >  CellDescPtr;

	RandomStructure(
		const size_t numToGenerate,
		const ::sstbx::build_cell::ICrystalStructureGenerator &           structureGenerator,
    const StructureDescPtr & structureDescription = StructureDescPtr(),
    const CellDescPtr &      cellDescription      = CellDescPtr());

  // From StartBlock ///
	virtual void start();
  // End from StartBlock


  // From PipeBlock //
	virtual void in(::spipe::common::StructureData & data);
  // End from PipeBlock

private:

  virtual void initDescriptions();

	const size_t myNumToGenerate;

  /** Should the block use the structure description found in shared data */
  const bool  myUseSharedDataStructureDesc;

  /** Should the block use the cell description found in shared data */
  const bool  myUseSharedDataCellDesc;

  ::boost::shared_ptr<const ::sstbx::build_cell::StructureDescription>            myStructureDescription;
  ::boost::shared_ptr<const ::sstbx::build_cell::RandomCellDescription<double> >  myCellDescription;

	const ::sstbx::build_cell::ICrystalStructureGenerator &	myStructureGenerator;

};


}
}

#endif /* RANDOM_STRUCTURE_H */
