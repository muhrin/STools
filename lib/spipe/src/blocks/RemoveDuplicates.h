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

#include <spl/utility/UniqueStructureSet.h>
#include <spl/utility/UtilityFwd.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace common {
class Structure;
}
}

namespace spipe {
namespace blocks {

class RemoveDuplicates : public SpPipeBlock, ::boost::noncopyable
{
public:

  RemoveDuplicates(::spl::utility::IStructureComparatorPtr comparator);
  RemoveDuplicates(const ::spl::utility::IStructureComparator & comparator);

	virtual void in(::spipe::common::StructureData & data);

  // From Block /////////////////////////
	virtual void pipelineFinishing();
  // End from Block ///////////////////

private:
  typedef spl::utility::UniqueStructureSet<StructureDataHandle> StructureSet;

	StructureSet	myStructureSet;
};

}
}

#endif /* REMOVE_DUPLICATES_H */
