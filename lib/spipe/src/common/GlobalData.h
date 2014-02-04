/*
 * GlobalData.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef GLOBAL_DATA_H
#define GLOBAL_DATA_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/filesystem.hpp>

#include <spl/io/StructureReadWriteManager.h>
#include <spl/utility/HeterogeneousMap.h>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace common {

class GlobalData
{
public:
  GlobalData();

  const std::string &
  getSeedName() const;
  void
  setSeedName(const std::string & seedName);

  spl::utility::HeterogeneousMap objectsStore;

  spl::io::StructureReadWriteManager &
  getStructureIo();

  const boost::filesystem::path &
  getWorkingDir() const;
  void
  setWorkingDir(const boost::filesystem::path & workingDir);

private:
  void
  reset();

  boost::filesystem::path myWorkingDir;
  std::string mySeedName;
  spl::io::StructureReadWriteManager myStructureIoManager;
};

}
}

#endif /* GLOBAL_DATA_H */
