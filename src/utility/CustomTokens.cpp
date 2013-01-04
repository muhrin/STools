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

EnergyToken::EnergyToken(
  const ::std::string & name,
  const ::std::string & symbol,
  const double relativeTo,
  const ::std::string & defaultFormatString,
  const bool usePerAtom):
TypedToken<double>(name, symbol, defaultFormatString),
myRelativeTo(relativeTo),
myUsePerAtom(usePerAtom)
{ 
}

EnergyToken::EnergyToken(
  const ::std::string & name,
  const ::std::string & symbol,
  const ::std::string & defaultFormatString,
  const bool usePerAtom):
TypedToken<double>(name, symbol, defaultFormatString),
myRelativeTo(0.0),
myUsePerAtom(usePerAtom)
{ 
}

void EnergyToken::setRelativeEnergy(const double relativeEnergy)
{
  myRelativeTo = relativeEnergy;
}

EnergyToken::StructureValue
EnergyToken::doGetValue(const ::sstbx::common::Structure & structure ) const
{
  StructureValue relativeEnergy;
  const double * const energy = structure.getProperty(structure_properties::general::ENERGY_INTERNAL);

  if(energy)
    relativeEnergy.reset(
    myUsePerAtom ? *energy / structure.getNumAtoms() : *energy
    - myRelativeTo);

  return relativeEnergy;
}

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
