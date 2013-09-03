/*
 * FindSymmetryGroup.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/FindSymmetryGroup.h"

#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/scoped_array.hpp>

#include <armadillo>

#include <spl/analysis/SpaceGroup.h>
#include <spl/common/AtomSpeciesId.h>
#include <spl/common/Structure.h>
#include <spl/common/UnitCell.h>

// Local includes
#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {
namespace common = ::spipe::common;
namespace ssc = ::spl::common;
namespace ssa = ::spl::analysis;
namespace structure_properties = ssc::structure_properties;

FindSymmetryGroup::FindSymmetryGroup():
Block("Determine space group")
{}

void FindSymmetryGroup::in(common::StructureData * const data)
{
  ssc::Structure * const structure = data->getStructure();

  if(structure && structure->getUnitCell())
  {
    ssa::space_group::SpacegroupInfo sgInfo;
    ssa::space_group::getSpacegroupInfo(sgInfo, *structure);

    structure->setProperty(
      structure_properties::general::SPACEGROUP_NUMBER,
      sgInfo.number
    );

    structure->setProperty(
      structure_properties::general::SPACEGROUP_SYMBOL,
      sgInfo.iucSymbol
    );
  }

  out(data);
}

}
}
