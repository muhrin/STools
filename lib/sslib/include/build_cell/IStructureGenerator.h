/*
 * IStructureGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_STRUCTURE_GENERATOR_H
#define I_STRUCTURE_GENERATOR_H

// INCLUDES /////////////////////////////////
#include <utility>

#include "build_cell/StructureGenerationOutcome.h"
#include "common/Types.h"

// FORWARD DECLARES //////////////////////////
namespace sstbx {
namespace build_cell {
class StructureDescription;
}
namespace common {
class Structure;
}
}

namespace sstbx {
namespace build_cell {

class IStructureGenerator
{
public:

  typedef ::std::pair<StructureGenerationOutcome::Value, ::sstbx::common::StructurePtr> Result;

  /** Generate a new structure */
  //virtual Result generateStructure() = 0;

  virtual Result generateStructure(const StructureDescription &  strDesc) const = 0;

  virtual ~IStructureGenerator() {}
};

}
}

#endif /* I_STRUCTURE_GENERATOR_H */
