/*
 * RemoveDuplicates.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef REMOVE_DUPLICATES_H
#define REMOVE_DUPLICATES_H

// INCLUDES /////////////////////////////////////////////

#include "pipelib/AbstractSimpleBlock.h"

#include <map>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
	namespace common
	{
		class Structure;
	}
	namespace utility
	{
		class IStructureSet;
	}
}

namespace spipe { namespace blocks {

class RemoveDuplicates : public pipelib::AbstractSimpleBlock<StructureDataTyp, SharedDataTyp>
{
public:
	RemoveDuplicates(sstbx::utility::IStructureSet & structureSet);

	virtual void in(::spipe::common::StructureData & data);

  // From Block /////////////////////////
	virtual void pipelineFinishing();
  // End from Block ///////////////////

private:

	typedef ::std::map<
		::sstbx::common::Structure *,
		::spipe::common::StructureData *> StructureDataMap;

	sstbx::utility::IStructureSet & myStructureSet;

	StructureDataMap	myStructureDataMap;

};

}}

#endif /* REMOVE_DUPLICATES_H */
