/*
 * CustomTokens.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/CustomTokens.h"

#include <sstream>


#include <spl/analysis/SpaceGroup.h>
#include <spl/common/AtomsFormula.h>
#include <spl/common/Structure.h>
#include <spl/common/StructureProperties.h>
#include <spl/io/BoostFilesystem.h>

// NAMESPACES ////////////////////////////////

namespace fs = ::boost::filesystem;
namespace ssa = ::spl::analysis;
namespace ssc = ::spl::common;
namespace ssio = ::spl::io;
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
EnergyToken::doGetValue(const ::spl::common::Structure & structure ) const
{
  StructureValue relativeEnergy;
  if(myUsePerAtom && structure.getNumAtoms() == 0)
    return relativeEnergy;

  const double * const energy = structure.getProperty(structure_properties::general::ENERGY_INTERNAL);
  if(energy)
    relativeEnergy.reset(
    (myUsePerAtom ? *energy / structure.getNumAtoms() : *energy)
    - myRelativeTo);

  return relativeEnergy;
}

FormulaToken::FormulaToken(
  const ::std::string & name,
  const ::std::string & symbol,
  const ::std::string & defaultFormatString
):
TypedToken< ::std::string>(name, symbol, defaultFormatString)
{}

::boost::optional< ::std::string>
FormulaToken::doGetValue(const ::spl::common::Structure & structure) const
{
  ::std::stringstream ss;
  ss << structure.getComposition();
  return ss.str();
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
  if(structure.getNumAtoms() == 0)
    return energyPerAtom;
  
  const double * const value = structure.getProperty(structure_properties::general::ENERGY_INTERNAL);
  if(value)
    energyPerAtom.reset(*value / structure.getNumAtoms());

  return energyPerAtom;
}

OptionalUInt getNumAtoms(const ::spl::common::Structure & structure)
{
  return structure.getNumAtoms();
}

::boost::optional< ::std::string>
getSpaceGroupSymbol(const ::spl::common::Structure & structure)
{
  ::boost::optional< ::std::string> spgroup;

  const ::std::string * const spgroupSymbol = structure.getProperty(structure_properties::general::SPACEGROUP_SYMBOL);

  if(spgroupSymbol && !spgroupSymbol->empty() && (*spgroupSymbol) != "P1")
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
getSpaceGroupNumber(const ::spl::common::Structure & structure)
{
  ::boost::optional<unsigned int> spgroup;

  const unsigned int * const spgroupNumber = structure.getProperty(structure_properties::general::SPACEGROUP_NUMBER);

  if(spgroupNumber && (*spgroupNumber) != 1)
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
getRelativeLoadPath(const ::spl::common::Structure & structure)
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
