/*
 * PotentialGo.h
 * Potential Geometry Optimisation
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef POTENTIAL_GO_H
#define POTENTIAL_GO_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include "pipelib/AbstractSimpleBlock.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
	namespace potential
	{
		class IGeomOptimiser;
	}
}


namespace spipe { namespace blocks {

class PotentialGo : public pipelib::AbstractSimpleBlock<StructureDataTyp, SharedDataTyp>
{
public:
	PotentialGo(::sstbx::potential::IGeomOptimiser & optimiser);

	virtual void in(spipe::common::StructureData & data);

private:
	const sstbx::potential::IGeomOptimiser & myOptimiser;
};

}}

#endif /* POTENTIAL_GO_H */
