/*
 * PipeFunctions.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "PipeFunctions.h"

#include <spl/utility/UtilFunctions.h>

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace ssu = ::spl::utility;

::std::string getOutputFileStem(const SharedDataType & shared, const GlobalDataType & global)
{
  ::std::string stem = global.getSeedName();
  if(!stem.empty() && !shared.getInstanceName().empty())
  {
    stem += "-";
    stem += shared.getInstanceName();
  }
  return stem;
}

::std::string generateStructureName(const GlobalDataType & global)
{
  ::std::string name = global.getSeedName();
  if(!name.empty())
    name += "-";
  name += ssu::generateUniqueName(4);
  return name;
}

}
}
