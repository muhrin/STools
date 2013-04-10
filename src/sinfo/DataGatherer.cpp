/*
 * DataGatherer.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "sinfo/DataGatherer.h"

#include <limits>

// From SSTbx
#include <common/Structure.h>
#include <common/StructureProperties.h>

// NAMESPACES ////////////////////////////////

namespace stools {
namespace sinfo {

namespace ssc = ::sstbx::common;
namespace structure_properties = ssc::structure_properties;

DataGatherer::DataGatherer():
myLowestEnergy(::std::numeric_limits<double>::max()),
myLowestEnergyPerAtom(::std::numeric_limits<double>::max()),
myLowestEnthalpy(::std::numeric_limits<double>::max()),
myLowestEnthalpyPerAtom(::std::numeric_limits<double>::max())
{}

void DataGatherer::gather(const ssc::Structure & structure)
{
  const double * const energy = structure.getProperty(structure_properties::general::ENERGY_INTERNAL);
  if(energy)
  {
    myLowestEnergy = ::std::min(myLowestEnergy, *energy);
    myLowestEnergyPerAtom = ::std::min(myLowestEnergyPerAtom, *energy / structure.getNumAtoms());
  }
  const double * const enthalpy = structure.getProperty(structure_properties::general::ENTHALPY);
  if(enthalpy)
  {
    myLowestEnthalpy = ::std::min(myLowestEnthalpy, *enthalpy);
    myLowestEnthalpyPerAtom = ::std::min(myLowestEnthalpyPerAtom, *enthalpy / structure.getNumAtoms());
  }
}

::boost::optional<double> DataGatherer::getLowestEnergy() const
{
  ::boost::optional<double> lowest;
  if(myLowestEnergy != ::std::numeric_limits<double>::max())
    lowest.reset(myLowestEnergy);
  return lowest;
}

::boost::optional<double> DataGatherer::getLowestEnergyPerAtom() const
{
  ::boost::optional<double> lowest;
  if(myLowestEnergyPerAtom != ::std::numeric_limits<double>::max())
    lowest.reset(myLowestEnergyPerAtom);
  return lowest;
}

::boost::optional<double> DataGatherer::getLowestEnthalpy() const
{
  ::boost::optional<double> lowest;
  if(myLowestEnthalpy != ::std::numeric_limits<double>::max())
    lowest.reset(myLowestEnthalpy);
  return lowest;
}

::boost::optional<double> DataGatherer::getLowestEnthalpyPerAtom() const
{
  ::boost::optional<double> lowest;
  if(myLowestEnthalpyPerAtom != ::std::numeric_limits<double>::max())
    lowest.reset(myLowestEnthalpyPerAtom);
  return lowest;
}

}
}
