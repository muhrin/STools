/*
 * RemoveDuplicates.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"

#include "blocks/RemoveDuplicates.h"

#include "common/StructureData.h"

// From SSTbx
#include <common/Structure.h>

#include <boost/foreach.hpp>

#include <map>

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssu = ::sstbx::utility;

RemoveDuplicates::RemoveDuplicates(sstbx::utility::IStructureComparator & comparator):
pipelib::Block<StructureDataTyp, SharedDataTyp, SharedDataTyp>("Remove duplicates"),
myStructureSet(comparator)
{}

void RemoveDuplicates::in(::spipe::common::StructureData & data)
{
  if(!data.getStructure())
  {
    out(data);
    return;
  }

  // Flag the data to say that we may want to use it again
  const StructureDataHandle handle = getRunner()->createDataHandle(data);
  const StructureSet::insert_return_type result = myStructureSet.insert(handle, *data.getStructure());

	if(result.second)
	{
    // Inserted
		data.timesFound.reset(1);
		out(data);
	}
	else
	{
    // Not inserted
    getRunner()->releaseDataHandle(handle);
		// The structure is not unique so discard it
		getRunner()->dropData(data);

		// Up the 'times found' counter on the original structure
		::spipe::common::StructureData & origStrData = getRunner()->getData(*result.first);
		if(origStrData.timesFound)
			origStrData.timesFound.reset(*origStrData.timesFound + 1);
		else
			origStrData.timesFound.reset(1);
	}
}

void RemoveDuplicates::pipelineFinishing()
{
	// Make sure we clean up any data we are holding on to
  BOOST_FOREACH(const StructureDataHandle & handle, myStructureSet)
	{
		getRunner()->releaseDataHandle(handle);
	}
	myStructureSet.clear();
}

}
}
