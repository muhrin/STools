/*
 * ICrystalStructureGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_CRYSTAL_STRUCTURE_GENERATOR_H
#define I_CRYSTAL_STRUCTURE_GENERATOR_H

// INCLUDES /////////////////////////////////
#include <utility>

#include "build_cell/StructureGenerationOutcome.h"
#include "common/Types.h"

// FORWARD DECLARES //////////////////////////
namespace sstbx
{
namespace build_cell
{
class RandomCellDescription;
class StructureDescription;
}
namespace common
{
class Structure;
}
}

namespace sstbx
{
namespace build_cell
{

class ICrystalStructureGenerator
{
public:

  typedef ::std::pair<StructureGenerationOutcome::Value, ::sstbx::common::StructurePtr> Result;

	/**
	 * Generate a new structure.
	 *
	 */
  virtual Result generateStructure(
    const StructureDescription &  strDesc,
    const RandomCellDescription & cellDesc) const = 0;
};

}
}

#endif /* I_CRYSTAL_STRUCTURE_GENERATOR_H */
