/*
 * CustomTokens.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/CustomTokens.h"

#include <common/Structure.h>
#include <common/StructureProperties.h>

// NAMESPACES ////////////////////////////////

namespace ssc = ::sstbx::common;
namespace structure_properties = ssc::structure_properties;

namespace stools {
namespace utility {
namespace functions {

typedef ::boost::optional<double> OptionalDouble;
typedef ::boost::optional< ::std::string> OptionalString;
typedef ::boost::optional<unsigned int> OptionalUInt;

OptionalString getName(const ssc::Structure & structure)
{
  OptionalString name;
  if(!structure.getName().empty())
    name.reset(structure.getName());

  return name;
}


OptionalDouble getVolume(const ssc::Structure & structure)
{
  OptionalDouble volume;
  if(structure.getUnitCell())
    volume.reset(structure.getUnitCell()->getVolume());

  return volume;
}

OptionalDouble getEnergyPerAtom(const ssc::Structure & structure)
{
  OptionalDouble energyPerAtom;
  
  const double * const value = structure.getProperty(structure_properties::general::ENERGY_INTERNAL);
  if(value)
    energyPerAtom.reset(*value / structure.getNumAtoms());

  return energyPerAtom;
}

OptionalUInt getNumAtoms(const ::sstbx::common::Structure & structure)
{
  return structure.getNumAtoms();
}

}
}
}
