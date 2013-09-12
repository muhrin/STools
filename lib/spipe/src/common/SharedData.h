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

#include <spl/build_cell/IStructureGenerator.h>
#include <spl/build_cell/BuildCellFwd.h>
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
  typedef ::spl::build_cell::IStructureGeneratorPtr IStructureGeneratorPtr;

  static const char DIR_SUBSTRING_DELIMITER[];

  SharedData();

  bool
  appendToOutputDirName(const ::std::string & toAppend);

  ::boost::filesystem::path
  getOutputPath() const;

  /**
   /* Get the output path for the pipeline that owns this shared data relative to
   /* the parent pipeline (or global data output path if there is no parent).
   /**/
  const ::boost::filesystem::path &
  getPipeRelativeOutputPath() const;

  const ::std::string &
  getInstanceName() const;

  ::spl::build_cell::IStructureGenerator *
  getStructureGenerator();
  const ::spl::build_cell::IStructureGenerator *
  getStructureGenerator() const;
  template< class T>
    void
    setStructureGenerator(SSLIB_UNIQUE_PTR(T) generator);

  ::spl::utility::HeterogeneousMap objectsStore;

private:
  void
  reset();

  IStructureGeneratorPtr myStructureGenerator;
  ::boost::filesystem::path myOutputDir;
  ::std::string myInstanceName;
};

template< class T>
  void
  SharedData::setStructureGenerator(SSLIB_UNIQUE_PTR(T) generator)
  {
    myStructureGenerator = generator;
  }

}
}

#endif /* SHARED_DATA_H */
