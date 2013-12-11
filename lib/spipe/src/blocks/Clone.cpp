/*
 * Clone.cpp
 *
 *  Created on: Dec 1, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/Clone.h"


// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {


Clone::Clone(const int times):
    Block("Clone"), myTimes(times), myGiveUniqueNames(true)
{
}

Clone::Clone(const int times, const bool giveUniqueNames):
    Block("Clone"), myTimes(times), myGiveUniqueNames(giveUniqueNames)
{
}

void
Clone::in(common::StructureData * const data)
{
  for(int i = 0; i < myTimes; ++i)
  {
    common::StructureData * const clone = getEngine()->createData();
    *clone = *data; // Copy
    out(clone);
  }

  out(data);
}

}
}
