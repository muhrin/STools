/*
 * LowestFreeEnergy.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef LOWEST_FREE_ENERGY_H
#define LOWEST_FREE_ENERGY_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <pipelib/AbstractSimpleBarrier.h>


// FORWARD DECLARATIONS ////////////////////////////////////


namespace spipe { namespace blocks {

class LowestFreeEnergy : public ::pipelib::AbstractSimpleBarrier<StructureDataTyp, SharedDataTyp>
{
public:

	LowestFreeEnergy();

	virtual void in(spipe::common::StructureData & data);

	virtual size_t release();

	virtual bool hasData() const;

private:

	::spipe::common::StructureData * myLowest;
};

}}

#endif /* LOWEST_FREE_ENERGY_H */
