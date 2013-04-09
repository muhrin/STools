/*
 * CustomTokens.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/CustomTokens.h"

// From SSTbx
#include <analysis/SpaceGroup.h>
#include <common/Structure.h>
#include <common/StructureProperties.h>
#include <io/BoostFilesystem.h>

// NAMESPACES ////////////////////////////////

namespace fs = ::boost::filesystem;
namespace ssa = ::sstbx::analysis;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
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
    (myUsePerAtom ? *energy / structure.getNumAtoms() : *energy)
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

OptionalDouble getVolumePerAtom(const ssc::Structure & structure)
{
  OptionalDouble volume;
  if(structure.getNumAtoms() > 0 && structure.getUnitCell())
    volume.reset(structure.getUnitCell()->getVolume() / structure.getNumAtoms());

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

::boost::optional< ::std::string>
getSpaceGroupSymbol(const ::sstbx::common::Structure & structure)
{
  ::boost::optional< ::std::string> spgroup;

  const ::std::string * const spgroupSymbol = structure.getProperty(structure_properties::general::SPACEGROUP_SYMBOL);

  if(spgroupSymbol)
    spgroup.reset(*spgroupSymbol);
  else if(structure.getUnitCell())
  {
    // Try to figure it out
    ssa::space_group::SpacegroupInfo sgInfo;
    ssa::space_group::getSpacegroupInfo(sgInfo, structure);
    spgroup.reset(sgInfo.iucSymbol);
  }

  return spgroup;
}

::boost::optional<unsigned int>
getSpaceGroupNumber(const ::sstbx::common::Structure & structure)
{
  ::boost::optional<unsigned int> spgroup;

  const unsigned int * const spgroupNumber = structure.getProperty(structure_properties::general::SPACEGROUP_NUMBER);

  if(spgroupNumber)
    spgroup.reset(*spgroupNumber);
  else if(structure.getUnitCell())
  {
    // Try to figure it out
    ssa::space_group::SpacegroupInfo sgInfo;
    ssa::space_group::getSpacegroupInfo(sgInfo, structure);
    spgroup.reset(sgInfo.number);
  }

  return spgroup;
}

::boost::optional<ssio::ResourceLocator>
getRelativeLoadPath(const ::sstbx::common::Structure & structure)
{
  ::boost::optional<ssio::ResourceLocator> loadLocator;

  const ssio::ResourceLocator * const absPath = structure.getProperty(structure_properties::io::LAST_ABS_FILE_PATH);
  if(absPath)
  {
    loadLocator.reset(*absPath);
    loadLocator->makeRelative(fs::current_path());
  }

  return loadLocator;
}

}
}
}
