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
#include <SSLib.h>
#include <build_cell/AtomsDescription.h>
#include <build_cell/ConstStructureDescriptionVisitor.h>
#include <build_cell/IStructureGenerator.h>
#include <common/Constants.h>
#include <common/Structure.h>
#include <common/Types.h>

// Local includes
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;

RandomStructure::RandomStructure(
  const ::sstbx::build_cell::IStructureGenerator &   structureGenerator,
  const OptionalUInt numToGenerate,
  const ::boost::shared_ptr<const ::sstbx::build_cell::StructureDescription > & structureDescription):
SpBlock("Generate Random structures"),
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
  const unsigned int numToGenerate = myNumToGenerate ? *myNumToGenerate : 100;
	
  initDescriptions();
  for(size_t i = 0; i < numToGenerate; ++i)
  {
	  // Create the random structure
    ssc::StructurePtr str = myStructureGenerator.generateStructure(*myStructureDescription, getRunner()->memory().global().getSpeciesDatabase());

	  if(str.get())
	  {
      StructureData & data = getRunner()->createData();
		  data.setStructure(str);

		  // Build up the name
			std::stringstream ss;
			ss << ::spipe::common::generateUniqueName() << "-" << i;
			data.getStructure()->setName(ss.str());

		  // Send it down the pipe
		  out(data);
	  }
  }	
}


void RandomStructure::in(::spipe::common::StructureData & data)
{
  initDescriptions();

	// Create the random structure
  ssc::StructurePtr str = myStructureGenerator.generateStructure(*myStructureDescription, getRunner()->memory().global().getSpeciesDatabase());

	if(str.get())
	{
		data.setStructure(str);

		// Build up the name
		if(!data.getStructure()->getName().empty())
		{
			std::stringstream ss;
			ss << ::spipe::common::generateUniqueName();
			data.getStructure()->setName(ss.str());
		}

		// Send it down the pipe
		out(data);
	}
	else
		getRunner()->dropData(data);
}

void RandomStructure::initDescriptions()
{
  const common::SharedData & sharedDat = getRunner()->memory().shared();
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
