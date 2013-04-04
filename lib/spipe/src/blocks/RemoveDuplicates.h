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

#include <boost/noncopyable.hpp>

#include <pipelib/pipelib.h>

#include <utility/UniqueStructureSet.h>
#include <utility/UtilityFwd.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace common {
class Structure;
}
}

namespace spipe {
namespace blocks {

class RemoveDuplicates : public SpPipeBlock, ::boost::noncopyable
{
public:

  RemoveDuplicates(::sstbx::utility::IStructureComparatorPtr comparator);
  RemoveDuplicates(const ::sstbx::utility::IStructureComparator & comparator);

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
