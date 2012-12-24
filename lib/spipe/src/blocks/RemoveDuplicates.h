/*
 * RemoveDuplicates.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef REMOVE_DUPLICATES_H
#define REMOVE_DUPLICATES_H

// INCLUDES /////////////////////////////////////////////
#include <map>

#include <pipelib/pipelib.h>

#include <utility/UniqueStructureSet.h>
#include <utility/IStructureComparator.h>

#include "PipeLibTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace common {
class Structure;
}
}

namespace spipe {
namespace blocks {

class RemoveDuplicates : public pipelib::PipeBlock<StructureDataTyp, SharedDataTyp, SharedDataTyp>
{
public:
	RemoveDuplicates(sstbx::utility::IStructureComparator & comparator);

	virtual void in(::spipe::common::StructureData & data);

  // From Block /////////////////////////
	virtual void pipelineFinishing();
  // End from Block ///////////////////

private:
  typedef sstbx::utility::UniqueStructureSet<StructureDataHandle> StructureSet;

	StructureSet	myStructureSet;
};

}
}

#endif /* REMOVE_DUPLICATES_H */
