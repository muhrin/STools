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

#include <spl/SSLib.h>
#include <spl/common/Structure.h>
#include <spl/common/Types.h>
#include <spl/io/BoostFilesystem.h>
#include <spl/io/IStructureReader.h>
#include <spl/utility/HeterogeneousMap.h>

// FORWARD DECLARATIONS ////////////////////////////////////
namespace spl {
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
  spl::common::Structure *
  getStructure() const;
  spl::common::Structure &
  setStructure(::spl::common::types::StructurePtr structure);
  spl::common::Structure &
  setStructure(::spl::io::StructuresContainer::auto_type structure);

  ::spl::io::ResourceLocator
  getRelativeSavePath(const ::boost::filesystem::path & relativeTo) const;

  ::spl::utility::HeterogeneousMap objectsStore;

private:
  ::spl::UniquePtr< ::spl::common::Structure>::Type myStructure;
};

}
}

#endif /* STRUCTURE_DATA_H */
