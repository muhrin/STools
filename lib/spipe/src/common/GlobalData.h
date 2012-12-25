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

// From SSTbx
#include <common/AtomSpeciesDatabase.h>
#include <io/StructureReadWriteManager.h>
#include <utility/HeterogeneousMap.h>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace common {

class GlobalData
{
public:
  static const char DIR_SUBSTRING_DELIMITER[];

  GlobalData();

  bool appendToOutputDirName(const ::std::string & toAppend);

  /**
  /* Get the output path for the pipeline that owns this shared data relative to
  /* the parent pipeline (or global data output path if there is not parent).
  /**/
  const ::boost::filesystem::path & getOutputPath() const;

  /**
  /* Get a stem filename for output being made by blocks within the pipeline that
  /* owns this shared data.
  /* Using this as the stem output filename allows output from a particular run
  /* through the pipeline to be easily identified.
  /*/
  const ::boost::filesystem::path & getOutputFileStem() const;

  ::sstbx::common::AtomSpeciesDatabase & getSpeciesDatabase();
  const ::sstbx::common::AtomSpeciesDatabase & getSpeciesDatabase() const;

  ::sstbx::utility::HeterogeneousMap  objectsStore;

  ::sstbx::io::StructureReadWriteManager & getStructureIo();

private:

  void reset();

  ::sstbx::common::AtomSpeciesDatabase  mySpeciesDatabase;
  ::boost::filesystem::path             myOutputDir;
  ::boost::filesystem::path             myOutputFileStem;
  ::sstbx::io::StructureReadWriteManager myStructureIoManager;
};


}
}

#endif /* GLOBAL_DATA_H */
