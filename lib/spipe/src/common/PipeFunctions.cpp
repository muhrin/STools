/*
 * PipeFunctions.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "PipeFunctions.h"

#include <utility/UtilFunctions.h>

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace ssu = ::sstbx::utility;

::std::string getOutputFileStem(const MemoryAccessType & memory)
{
  ::std::string stem = memory.global().getSeedName();
  if(!stem.empty() && !memory.shared().getInstanceName().empty())
    stem += "-";
  stem += memory.shared().getInstanceName();
  return stem;
}

::std::string generateStructureName(const MemoryAccessType & memory)
{
  ::std::string name = memory.global().getSeedName();
  if(!name.empty())
    name += "-";
  name += ssu::generateUniqueName(4);
  return name;
}

}
}
