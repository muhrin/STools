/*
 * RandomStructure.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/RandomStructure.h"

#include "common/StructureData.h"
#include "common/UtilityFunctions.h"

// From SSTbx
#include <build_cell/ICrystalStructureGenerator.h>
#include <build_cell/RandomCellDescription.h>
#include <build_cell/RandomCellGenerator.h>
#include <common/AbstractFmidCell.h>
#include <common/Structure.h>

// From PipelineLib
#include <pipelib/IPipeline.h>

// Local includes
#include "common/SharedData.h"

// NAMESPACES ////////////////////////////////


namespace spipe { namespace blocks {

RandomStructure::RandomStructure(
	const size_t numToGenerate,
  const ::sstbx::build_cell::ICrystalStructureGenerator &   structureGenerator,
  const ::boost::shared_ptr<const ::sstbx::build_cell::StructureDescription > & structureDescription,
  const ::boost::shared_ptr<const ::sstbx::build_cell::RandomCellDescription<double> > & cellDescription):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Random structures"),
myNumToGenerate(numToGenerate),
myStructureGenerator(structureGenerator),
myStructureDescription(structureDescription),
myCellDescription(cellDescription),
myUseSharedDataStructureDesc(!structureDescription.get()),
myUseSharedDataCellDesc(!cellDescription.get())
{
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
	sstbx::common::Structure * const str =
    myStructureGenerator.generateStructure(*myStructureDescription, *myCellDescription);

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

  if(myUseSharedDataCellDesc)
  {
    if(sharedDat.cellDescription.get())
    {
      myCellDescription = sharedDat.cellDescription;
    }
    else
    {
      // TODO: Throw some kind of exception, or emit error
    }
  }
}

}
}
