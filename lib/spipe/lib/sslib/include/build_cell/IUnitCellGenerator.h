/*
 * IUnitCellGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_UNIT_CELL_GENERATOR_H
#define I_UNIT_CELL_GENERATOR_H

// INCLUDES ////////////
#include "build_cell/BuildCellFwd.h"
#include "common/Types.h"

namespace sstbx {
// FORWARD DECLARATIONS ///////
namespace common {
class UnitCell;
}

namespace build_cell {
class GenerationOutcome;
class StructureContents;

class IUnitCellGenerator
{
public:
  virtual ~IUnitCellGenerator() {}

  virtual GenerationOutcome generateCell(common::UnitCellPtr & cellOut) const = 0;
  virtual GenerationOutcome generateCell(common::UnitCellPtr & cellOut, const StructureContents & contents) const = 0;

  virtual IUnitCellGeneratorPtr clone() const = 0;
};

}
}

#endif /* I_UNIT_CELL_GENERATOR_H */
