/*
 * LowestFreeEnergy.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/LowestFreeEnergy.h"

#include <common/Structure.h>
#include <common/StructureData.h>

#include <pipelib/pipelib.h>

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssc = ::sstbx::common;
namespace structure_properties = ssc::structure_properties;

LowestFreeEnergy::LowestFreeEnergy():
pipelib::Block<StructureDataTyp, SharedDataTyp, SharedDataTyp>("Lowest free energy"),
myLowest(NULL)
{}

void LowestFreeEnergy::in(spipe::common::StructureData & data)
{
	if(myLowest)
	{
    ssc::Structure * structure = data.getStructure();
    double * internalEnergy = structure->getProperty(structure_properties::general::ENERGY_INTERNAL);
		if(internalEnergy &&
      *internalEnergy < *myLowest->getStructure()->getProperty(structure_properties::general::ENERGY_INTERNAL))
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
