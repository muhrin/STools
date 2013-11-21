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

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include <armadillo>

#include <spl/build_cell/BuildCellFwd.h>
#include <spl/build_cell/GenerationSettings.h>
#include <spl/io/BoostFilesystem.h>
#include <spl/utility/HeterogeneousMap.h>

// Local includes
#include "PipeLibTypes.h"
#include "utility/DataTable.h"
#include "common/CommonData.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace common {

class SharedData
{
public:
  static const char DIR_SUBSTRING_DELIMITER[];

  SharedData();

  void
  setOutputDir(const ::boost::filesystem::path & path);
  bool
  appendToOutputDirName(const ::std::string & toAppend);
  ::boost::filesystem::path
  getOutputPath() const;

  const ::std::string &
  getInstanceName() const;

  ::spl::utility::HeterogeneousMap objectsStore;

private:
  void
  reset();

  ::boost::filesystem::path myOutputDir;
  ::std::string myInstanceName;
};



}
}

#endif /* SHARED_DATA_H */
