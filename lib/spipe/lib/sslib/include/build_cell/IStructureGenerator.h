/*
 * IStructureGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_STRUCTURE_GENERATOR_H
#define I_STRUCTURE_GENERATOR_H

// INCLUDES /////////////////////////////////

#include "common/Types.h"

namespace sstbx {

// FORWARD DECLARES //////////////////////////
namespace common {
class AtomSpeciesDatabase;
class Structure;
}

namespace build_cell {
class GenerationOutcome;

class IStructureGenerator
{
public:

  virtual ~IStructureGenerator() {}

  virtual GenerationOutcome generateStructure(
    common::StructurePtr & structureOut,
    const common::AtomSpeciesDatabase & speciesDb
  ) const = 0;
};

}
}

#endif /* I_STRUCTURE_GENERATOR_H */
