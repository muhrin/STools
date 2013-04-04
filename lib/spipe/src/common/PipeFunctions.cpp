/*
 * PipeFunctions.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "PipeFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

::std::string getOutputFileStem(const MemoryAccessType & memory)
{
  ::std::string stem = memory.global().getSeedName();
  if(!stem.empty() && !memory.shared().getInstanceName().empty())
    stem += "-";
  stem += memory.shared().getInstanceName();
  return stem;
}

}
}
