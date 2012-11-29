/*
 * StructureData.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef STRUCTURE_DATA_H
#define STRUCTURE_DATA_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/optional.hpp>

#include <armadillo>

// From SSLib
#include <SSLib.h>
#include <common/Types.h>
#include <utility/BoostFilesystem.h>
#include <utility/HeterogeneousMap.h>

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx {
namespace common {
class Structure;
}
}

namespace spipe {
namespace common {

class StructureData
{
public:

	sstbx::common::Structure * getStructure() const;
  sstbx::common::Structure & setStructure(::sstbx::UniquePtr< ::sstbx::common::Structure>::Type structure);

  /**
  /* Get the path to where this structure was last saved relative to the output path
  /* of a given structure pipe.
  /**/
  ::boost::filesystem::path getRelativeSavePath(const ::spipe::SpPipelineTyp & pipeline) const;

  ::sstbx::utility::HeterogeneousMap  objectsStore;

private:

  ::sstbx::UniquePtr< ::sstbx::common::Structure>::Type   myStructure;
};


}}

#endif /* STRUCTURE_DATA_H */
