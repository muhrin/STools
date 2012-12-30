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
myLowestEnergy(::std::numeric_limits<double>::min())
{}

void DataGatherer::gather(const ssc::Structure & structure)
{
  const double * const energy = structure.getProperty(structure_properties::general::ENERGY_INTERNAL);
  if(energy)
  {
    myLowestEnergy = ::std::min(myLowestEnergy, *energy);
    myLowestEnergyPerAtom = ::std::min(myLowestEnergyPerAtom, *energy / structure.getNumAtoms());
  }
}

::boost::optional<double> DataGatherer::getLowestEnergy() const
{
  ::boost::optional<double> lowest;
  if(myLowestEnergy != ::std::numeric_limits<double>::min())
    lowest.reset(myLowestEnergy);
  return lowest;
}

::boost::optional<double> DataGatherer::getLowestEnergyPerAtom() const
{
  ::boost::optional<double> lowest;
  if(myLowestEnergyPerAtom != ::std::numeric_limits<double>::min())
    lowest.reset(myLowestEnergyPerAtom);
  return lowest;
}

}
}
