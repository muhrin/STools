/*
 * RandomStructure.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/RandomStructure.h"

#include <boost/optional.hpp>

// From SSTbx
#include <build_cell/AtomsDescription.h>
#include <build_cell/ConstStructureDescriptionVisitor.h>
#include <build_cell/IStructureGenerator.h>
#include <common/Constants.h>
#include <common/Structure.h>
#include <common/Types.h>

// From PipelineLib
#include <pipelib/IPipeline.h>

// Local includes
#include "common/SharedData.h"
#include "common/StructureData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;

RandomStructure::RandomStructure(
	const size_t numToGenerate,
  const ::sstbx::build_cell::IStructureGenerator &   structureGenerator,
  const ::boost::shared_ptr<const ::sstbx::build_cell::StructureDescription > & structureDescription):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Random structures"),
myNumToGenerate(numToGenerate),
myStructureGenerator(structureGenerator),
myStructureDescription(structureDescription),
myUseSharedDataStructureDesc(!structureDescription.get())
{
}

void RandomStructure::pipelineStarting()
{
  // TODO: Put structure description initialisation stuff here
}

void RandomStructure::start()
{
	using ::spipe::common::StructureData;
	for(size_t i = 0; i < myNumToGenerate; ++i)
	{
		StructureData & data = myPipeline->newData();

		// Build up the name
		std::stringstream ss;
		ss << ::spipe::common::generateUniqueName() << "-" << i;
		data.name.reset(ss.str());

		in(data);
	}
}


void RandomStructure::in(::spipe::common::StructureData & data)
{
  initDescriptions();

	// Create the random structure
	const sstbx::common::StructurePtr str = myStructureGenerator.generateStructure(*myStructureDescription).second;

	if(str)
	{
		data.setStructure(str);

		// Build up the name
		if(!data.name)
		{
			std::stringstream ss;
			ss << ::spipe::common::generateUniqueName();
			data.name.reset(ss.str());
		}

		// Send it down the pipe
		myOutput->in(data);
	}
	else
	{
		myPipeline->dropData(data);
	}
}

void RandomStructure::initDescriptions()
{
  const common::SharedData & sharedDat = myPipeline->getSharedData();
  if(myUseSharedDataStructureDesc)
  {
    if(sharedDat.structureDescription.get())
    {
      myStructureDescription = sharedDat.structureDescription;
    }
    else
    {
      // TODO: Throw some kind of exception, or emit error
    }
  }
}

}
}
