/*
 * NiggliReduction.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef NIGGLI_REDUCTION_H
#define NIGGLI_REDUCTION_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/noncopyable.hpp>

#include <pipelib/pipelib.h>

#include "PipeLibTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace blocks {

class NiggliReduction : public SpPipeBlock, ::boost::noncopyable
{
public:

	NiggliReduction();

	virtual void in(::spipe::StructureDataTyp & data);
};


}
}

#endif /* NIGGLI_REDUCTION_H */
