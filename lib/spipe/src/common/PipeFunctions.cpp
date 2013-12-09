/*
 * PipeFunctions.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "PipeFunctions.h"

#include <spl/io/BoostFilesystem.h>
#include <spl/utility/UtilFunctions.h>

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace fs = ::boost::filesystem;
namespace ssu = ::spl::utility;

::std::string getOutputFileStem(const SharedData & shared, const GlobalData & global)
{
  ::std::string stem = global.getSeedName();
  if(!stem.empty() && !shared.getInstanceName().empty())
  {
    stem += "-";
    stem += shared.getInstanceName();
  }
  return stem;
}

::boost::filesystem::path
getWorkingDir(const SharedData & shared, const GlobalData & global)
{
  fs::path dir = shared.getWorkingDir();
  if(!dir.empty() && ::spl::io::isAbsolute(dir))
    return dir;

  return global.getWorkingDir() / dir;
}

::std::string generateStructureName(const GlobalData & global)
{
  ::std::string name = global.getSeedName();
  if(!name.empty())
    name += "-";
  name += ssu::generateUniqueName(4);
  return name;
}

}
}
