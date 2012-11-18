/*
 * LowestFreeEnergy.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/LowestFreeEnergy.h"

#include "common/StructureData.h"


#include <pipelib/IPipeline.h>

// NAMESPACES ////////////////////////////////


namespace spipe { namespace blocks {

LowestFreeEnergy::LowestFreeEnergy():
pipelib::Block<StructureDataTyp, SharedDataTyp>("Lowest free energy"),
myLowest(NULL)
{}

void LowestFreeEnergy::in(spipe::common::StructureData & data)
{
	if(myLowest)
	{
		if(data.enthalpy < myLowest->enthalpy)
		{
			// Unflag and delete the old lowest
			myPipeline->unflagData(*this, *myLowest);
			myPipeline->dropData(*myLowest);

			myPipeline->flagData(*this, data);
			myLowest = &data;
		}
		else
		{
			myPipeline->dropData(data);
		}
	}
	else
	{
		// Flag the data to say that we will want to use it again
		myPipeline->flagData(*this, data);
		myLowest = &data;
	}
}

size_t LowestFreeEnergy::release()
{
	if(myLowest)
	{
		myPipeline->unflagData(*this, *myLowest);
		myOutput->in(*myLowest);
		myLowest = NULL;
		return 1;
	}
	else
	{
		return 0;
	}
}

bool LowestFreeEnergy::hasData() const
{
	return myLowest != NULL;
}

}}
