/*
 * SharedData.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/filesystem.hpp>

#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/utility/HeterogeneousMap.h>

// Local includes
#include "common/CommonData.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace common {

class SharedData
{
public:
  SharedData();

  const std::string &
  getInstanceName() const;

  spl::common::AtomSpeciesDatabase &
  getSpeciesDatabase();
  const spl::common::AtomSpeciesDatabase &
  getSpeciesDatabase() const;

  spl::utility::HeterogeneousMap objectsStore;

  const boost::filesystem::path &
  getWorkingDir() const;
  void
  setWorkingDir(const boost::filesystem::path & workingDir);

private:
  void
  reset();

  spl::common::AtomSpeciesDatabase mySpeciesDb;
  boost::filesystem::path myWorkingDir;
  std::string myInstanceName;
};



}
}

#endif /* SHARED_DATA_H */
