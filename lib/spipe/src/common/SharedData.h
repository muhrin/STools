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

#include <common/AtomSpeciesDatabase.h>
#include <io/BoostFilesystem.h>

// Local includes
#include "PipeLibTypes.h"
#include "utility/DataTable.h"
#include "common/CommonData.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace build_cell {
class StructureDescription;
}
}

namespace spipe {
namespace common {

class SharedData
{
public:

  typedef ::boost::shared_ptr<sstbx::build_cell::StructureDescription> StructureDescriptionPtr;
  typedef ::boost::shared_ptr<const sstbx::build_cell::StructureDescription> ConstStructureDescriptionPtr;

  static const char DIR_SUBSTRING_DELIMITER[];

  SharedData();

  bool appendToOutputDirName(const ::std::string & toAppend);

  /**
  /* Get the output path for the pipeline that owns this shared data relative to
  /* the working directory where the code was executed.
  /**/
  ::boost::filesystem::path getOutputPath(const SpRunnerAccess & runner) const;

  /**
  /* Get the output path for the pipeline that owns this shared data relative to
  /* the parent pipeline (or global data output path if there is not parent).
  /**/
  const ::boost::filesystem::path & getPipeRelativeOutputPath() const;

  /**
  /* Get a stem filename for output being made by blocks within the pipeline that
  /* owns this shared data.
  /* Using this as the stem output filename allows output from a particular run
  /* through the pipeline to be easily identified.
  /*/
  const ::boost::filesystem::path & getOutputFileStem() const;

  StructureDescriptionPtr getStructureDescription();
  ConstStructureDescriptionPtr getStructureDescription() const;
  void setStructureDescription(StructureDescriptionPtr description);

  ::sstbx::common::AtomSpeciesDatabase & getSpeciesDatabase();
  const ::sstbx::common::AtomSpeciesDatabase & getSpeciesDatabase() const;


  ::std::string                       outputFilename;

  ::sstbx::utility::HeterogeneousMap  objectsStore;

  ::spipe::utility::DataTable          dataTable;

private:

  void reset();

  void buildOutputPathRecursive(::boost::filesystem::path & path, const SpRunnerAccess & runner) const;

  StructureDescriptionPtr structureDescription;
  ::sstbx::common::AtomSpeciesDatabase  mySpeciesDatabase;
  ::boost::filesystem::path             myOutputDir;
  ::boost::filesystem::path             myOutputFileStem;

};


}
}

#endif /* SHARED_DATA_H */
