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
#include <utility/UniqueStructureSet.h>

// From PipelineLib
#include <pipelib/IPipeline.h>

#include <boost/foreach.hpp>

#include <map>

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssc = ::sstbx::common;
namespace ssu = ::sstbx::utility;
namespace structure_properties = ssc::structure_properties;

RemoveDuplicates::RemoveDuplicates(ssu::UniqueStructureSet & structureSet):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Remove duplicates"),
myStructureSet(structureSet)
{}

void RemoveDuplicates::in(::spipe::common::StructureData & data)
{
  const std::pair<ssu::UniqueStructureSet::iterator, bool> result = myStructureSet.insert(data.getStructure());
  ssc::Structure * const structure = data.getStructure();

	if(result.second)
	{
		// Flag the data to say that we will want to use it again
		myPipeline->flagData(*this, data);
    myStructureDataMap.insert(StructureDataMap::value_type(*result.first, &data));
    structure->setProperty(structure_properties::searching::TIMES_FOUND, (unsigned int)1);

		myOutput->in(data);
	}
	else
	{
		// The structure is not unique so discard it
		myPipeline->dropData(data);

		// Up the 'times found' counter on the original structure
		StructureDataMap::iterator it = myStructureDataMap.find(*result.first);
		
		if(it == myStructureDataMap.end())
		{
			PASSERT(true);
		}

		::spipe::common::StructureData & origStrData = *it->second;
    unsigned int * timesFound =
      origStrData.getStructure()->getProperty(structure_properties::searching::TIMES_FOUND);
		if(timesFound)
		{
      origStrData.getStructure()->setProperty(
        structure_properties::searching::TIMES_FOUND,
        *timesFound + 1
      );
		}
		else
		{
      origStrData.getStructure()->setProperty(
        structure_properties::searching::TIMES_FOUND,
        (unsigned int)1
      );
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
