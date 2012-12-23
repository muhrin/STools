/*
 * LowestFreeEnergy.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/LowestFreeEnergy.h"

#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

LowestFreeEnergy::LowestFreeEnergy():
pipelib::Block<StructureDataTyp, SharedDataTyp, SharedDataTyp>("Lowest free energy"),
myLowest(NULL)
{}

void LowestFreeEnergy::in(spipe::common::StructureData & data)
{
	if(myLowest)
	{
		if(data.enthalpy < myLowest->enthalpy)
		{
			getRunner()->dropData(*myLowest);
			myLowest = &data;
		}
		else
			getRunner()->dropData(data);
	}
	else
		myLowest = &data;
}

size_t LowestFreeEnergy::release()
{
	if(myLowest)
	{
		out(*myLowest);
		myLowest = NULL;
		return 1;
	}
	else
		return 0;
}

bool LowestFreeEnergy::hasData() const
{
	return myLowest != NULL;
}

}
}
