/*
 * SeparateAtoms.cpp
 *
 *  Created on: Jan 23, 2014
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/SeparateAtoms.h"

#include <spl/build_cell/PointSeparator.h>
#include <spl/common/Structure.h>

// From local

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

SeparateAtoms::SeparateAtoms():
    Block("Separate atoms")
{
}

void
SeparateAtoms::in(StructureDataType * const data)
{
  static const spl::build_cell::PointSeparator SEPARATOR;

  if(data->getStructure())
  {
    spl::common::Structure * const structure = data->getStructure();
    spl::build_cell::SeparationData sepData(*structure,
        getEngine()->globalData().getSpeciesDatabase());
    if(SEPARATOR.separatePoints(&sepData))
      structure->setAtomPositions(sepData.points);
  }

  out(data);
}

}
}

