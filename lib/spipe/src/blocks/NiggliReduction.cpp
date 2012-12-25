/*
 * NiggliReduction.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/NiggliReduction.h"

#include "common/StructureData.h"

// From SSTbx

#include <common/Structure.h>
#include <common/UnitCell.h>

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssc = ::sstbx::common;

NiggliReduction::NiggliReduction():
SpBlock("Niggli reduction")
{}

void NiggliReduction::in(StructureDataType & data)
{
  ssc::UnitCell * const cell = data.getStructure()->getUnitCell();
  if(cell)
    cell->niggliReduce();

	out(data);
}

}
}

