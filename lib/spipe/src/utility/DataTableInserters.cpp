/*
 * DataTableInserters.cpp
 *
 *  Created on: Sep 12, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/DataTableInserters.h"

#include <vector>

#include <spl/common/Structure.h>
#include <spl/io/ResourceLocator.h>

#include "common/StructureData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace utility {

namespace splc = ::spl::common;
namespace splio = ::spl::io;
namespace structure_properties = ::spl::common::structure_properties;

bool
insertStructureInfoAndPotentialParams(const utility::DataTable::Key & key,
    const common::StructureData & structureData,
    const ::boost::filesystem::path & pathsRelativeTo, DataTable & table)
{
  if(!structureData.getStructure())
    return false;

  const ::std::vector< double> * const params = structureData.objectsStore.find(
      common::GlobalKeys::POTENTIAL_PARAMS);
  if(params)
  {
    // Update the table with the current parameters
    for(size_t i = 0; i < params->size(); ++i)
    {
      table.insert(key, "p_" + ::boost::lexical_cast< ::std::string>(i),
          common::getString((*params)[i]));
    }
  }

  const splc::Structure * const structure = structureData.getStructure();

  const double * const internalEnergy = structure->getProperty(
      structure_properties::general::ENERGY_INTERNAL);
  if(internalEnergy)
  {
    const double energy = *internalEnergy;
    table.insert(key, "energy", common::getString(energy));

    const size_t numAtoms = structure->getNumAtoms();
    table.insert(key, "energy/atom", common::getString(energy / numAtoms));
  }

  const splio::ResourceLocator locator = structureData.getRelativeSavePath(
      pathsRelativeTo);
  if(!locator.empty())
    table.insert(key, "lowest_path", locator.string());

  insertStructureProperty(key, "sg",
      structure_properties::general::SPACEGROUP_NUMBER, *structure, table);

  return true;
}

}
}
