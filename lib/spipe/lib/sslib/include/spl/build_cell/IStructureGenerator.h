/*
 * IStructureGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_STRUCTURE_GENERATOR_H
#define I_STRUCTURE_GENERATOR_H

// INCLUDES /////////////////////////////////

#include "spl/common/Types.h"

#include "spl/build_cell/GenerationOutcome.h"

namespace spl {

// FORWARD DECLARES //////////////////////////
namespace common {
class AtomSpeciesDatabase;
class Structure;
}

namespace build_cell {

class IStructureGenerator
{
public:

  virtual ~IStructureGenerator() {}

  virtual GenerationOutcome generateStructure(
    common::StructurePtr & structureOut,
    const common::AtomSpeciesDatabase & speciesDb
  ) = 0;
};

}
}

#endif /* I_STRUCTURE_GENERATOR_H */
