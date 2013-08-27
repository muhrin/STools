/*
 * MakeMap.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/MakeMap.h"

#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

MakeMap::MakeMap() :
    SpBlock("Make map")
{
}

void
MakeMap::in(spipe::common::StructureData & data)
{
  out(data);
}

}
}

