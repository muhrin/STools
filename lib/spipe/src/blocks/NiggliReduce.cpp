/*
 * NiggliReduce.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/NiggliReduce.h"

#include <spl/common/Structure.h>
#include <spl/common/UnitCell.h>

#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

namespace ssc = ::spl::common;

NiggliReduce::NiggliReduce() :
    Block("Niggli reduce")
{
}

void
NiggliReduce::in(common::StructureData * const data)
{
  ssc::UnitCell * const cell = data->getStructure()->getUnitCell();
  if(cell)
    cell->niggliReduce();

  out(data);
}

}
}

