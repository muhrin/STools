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
#include <io/BoostFilesystem.h>
#include <io/IStructureReader.h>
#include <utility/HeterogeneousMap.h>

#include "PipeLibTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx {
namespace common {
class Structure;
}
namespace io {
class ResourceLocator;
}
}

namespace spipe {
namespace common {

class StructureData
{
public:

  sstbx::common::Structure * getStructure() const;
  sstbx::common::Structure & setStructure(::sstbx::common::types::StructurePtr structure);
  sstbx::common::Structure & setStructure(::sstbx::io::StructuresContainer::auto_type structure);

  /**
  /* Get the path to where this structure was last saved relative to the output path
  /* of a given structure pipe.
  /**/
  ::sstbx::io::ResourceLocator getRelativeSavePath(const SpRunnerAccess & runner) const;

  ::sstbx::utility::HeterogeneousMap  objectsStore;

private:

  ::sstbx::UniquePtr< ::sstbx::common::Structure>::Type   myStructure;
};

}
}

#endif /* STRUCTURE_DATA_H */
