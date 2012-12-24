/*
 * DetermineSpaceGroup.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/DetermineSpaceGroup.h"

#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/scoped_array.hpp>

#include <armadillo>

extern "C"
{
#  include <spglib/spglib.h>
}

#include <analysis/SpaceGroup.h>
#include <common/AtomSpeciesId.h>
#include <common/Structure.h>
#include <common/UnitCell.h>

// Local includes
#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {
namespace common = ::spipe::common;
namespace ssc = ::sstbx::common;
namespace ssa = ::sstbx::analysis;
namespace structure_properties = ssc::structure_properties;

DetermineSpaceGroup::DetermineSpaceGroup():
::pipelib::Block<StructureDataTyp, SharedDataTyp, SharedDataTyp>("Determine space group")
{}

void DetermineSpaceGroup::in(StructureDataTyp & data)
{
  // Express structure data in form that spglib can work with
  ssc::Structure * const structure = data.getStructure();

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
