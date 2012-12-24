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

#include <boost/noncopyable.hpp>

#include <pipelib/pipelib.h>

#include "PipeLibTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////


namespace spipe {
namespace blocks {

class LowestFreeEnergy : public SpBarrier, ::boost::noncopyable
{
public:

	LowestFreeEnergy();

  // From Block /////////////////
	virtual void in(spipe::common::StructureData & data);
  // End from Block /////////////

  // From Barrier /////////////////
	virtual size_t release();
	virtual bool hasData() const;
  // End from Barrier //////////////

private:
	::spipe::common::StructureData * myLowest;
};

}
}

#endif /* LOWEST_FREE_ENERGY_H */
