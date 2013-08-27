/*
 * LowestFreeEnergy.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/LowestFreeEnergy.h"

#include <spl/common/Structure.h>

#include <pipelib/pipelib.h>

#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssc = ::spl::common;
namespace structure_properties = ssc::structure_properties;

LowestFreeEnergy::LowestFreeEnergy(const size_t keepTopN):
SpBlock("Lowest free energy"),
myKeepTopNMode(true),
myKeepTopN(keepTopN),
myKeepTopEnergyPercentage(0.0)
{}

LowestFreeEnergy::LowestFreeEnergy(const double keepTopEnergyPercentage):
SpBlock("Lowest free energy"),
myKeepTopNMode(false),
myKeepTopN(0),
myKeepTopEnergyPercentage(keepTopEnergyPercentage)
{}

void LowestFreeEnergy::in(spipe::common::StructureData & data)
{
  const ssc::Structure * const structure = data.getStructure();
  const double * const internalEnergy = structure->getProperty(structure_properties::general::ENERGY_INTERNAL);
  // Let anything that doesn't have an internal energy through
  if(internalEnergy == NULL)
  {
    out(data);
    return;
  }

  keep(data, *internalEnergy);
}

size_t LowestFreeEnergy::release()
{
  const size_t numReleased = myStructures.size();
  BOOST_FOREACH(Structures::reference structurePair, myStructures)
  {
    out(*structurePair.second);
  }
  myStructures.clear();
	return numReleased;
}

bool LowestFreeEnergy::hasData() const
{
	return !myStructures.empty();
}

void LowestFreeEnergy::keep(StructureData & structure, const double energy)
{
  if(myKeepTopNMode)
  {
    myStructures[energy] = &structure;
    if(myStructures.size() > myKeepTopN)
    {
      // There are already enough
      Structures::iterator removeStart = myStructures.begin();
      // Skip over the ones we want to keep
      for(size_t i = 0; i < myKeepTopN; ++i)
        ++removeStart;

      for(Structures::iterator it = removeStart, end = myStructures.end();
        it != end; ++it)
      {
        getRunner()->dropData(*it->second);
      }
      // Remove the entries from the container
      myStructures.erase(removeStart, myStructures.end());
    }
  }
  else
  {
    // Check if we have any structures yet
    if(myStructures.empty())
      myStructures[energy] = &structure;
    else
    {
      if(energy < myStructures.begin()->first)
        newLowest(structure, energy);
      else if(energy < getEnergyCutoff())
        myStructures[energy] = &structure;
    }
  }
}

void LowestFreeEnergy::newLowest(StructureData & structure, const double energy)
{
  myStructures[energy] = &structure;
  const double cutoff = getEnergyCutoff();

  // Go from the end erasing all entries greater than the new cutoff
  Structures::reverse_iterator it;

  // Erasing using reverse iterator is a bit annoying
  // see: http://stackoverflow.com/questions/1830158/how-to-call-erase-with-a-reverse-iterator
  while((it = myStructures.rbegin())->first > cutoff)
    myStructures.erase((++it).base());
}

double LowestFreeEnergy::getEnergyCutoff() const
{
  return myStructures.begin()->first * (1.0 - myKeepTopEnergyPercentage);
}

}
}
