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
#include <utility/IStructureSet.h>

// From PipelineLib
#include <pipelib/IPipeline.h>

#include <boost/foreach.hpp>

#include <map>

// NAMESPACES ////////////////////////////////


namespace spipe { namespace blocks {

RemoveDuplicates::RemoveDuplicates(sstbx::utility::IStructureSet & structureSet):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Remove duplicates"),
myStructureSet(structureSet)
{}

void RemoveDuplicates::in(::spipe::common::StructureData & data)
{
	const std::pair<sstbx::common::Structure *, bool> result = myStructureSet.insert(data.getStructure());

	if(result.second)
	{
		// Flag the data to say that we will want to use it again
		myPipeline->flagData(*this, data);
    myStructureDataMap.insert(StructureDataMap::value_type(result.first, &data));
		data.timesFound.reset(1);

		myOutput->in(data);
	}
	else
	{
		// The structure is not unique so discard it
		myPipeline->dropData(data);

		// Up the 'times found' counter on the original structure
		StructureDataMap::iterator it = myStructureDataMap.find(result.first);
		
		if(it == myStructureDataMap.end())
		{
			PASSERT(true);
		}

		::spipe::common::StructureData & origStrData = *it->second;
		if(origStrData.timesFound)
		{
			origStrData.timesFound.reset(*origStrData.timesFound + 1);
		}
		else
		{
			origStrData.timesFound.reset(1);
		}


	}
}

void RemoveDuplicates::pipelineFinishing()
{
	// Make sure we clean up any data we are holding on to
  BOOST_FOREACH(const StructureDataMap::value_type & pair, myStructureDataMap)
	{
		myPipeline->unflagData(*this, *pair.second);
	}
	myStructureDataMap.clear();
	myStructureSet.clear();
}

}}
