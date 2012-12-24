/*
 * MakeMap.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef MAKE_MAP_H
#define MAKE_MAP_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/noncopyable.hpp>

#include <pipelib/pipelib.h>

#include "PipeLibTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////


namespace spipe {
namespace blocks {

class MakeMap : public SpPipeBlock, ::boost::noncopyable
{
public:

	MakeMap();

	virtual void in(spipe::common::StructureData & data);
};

}
}

#endif /* MAKE_MAP_H */
