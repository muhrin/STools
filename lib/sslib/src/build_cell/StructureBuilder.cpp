/*
 * StructureBuilder.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////////
#include "build_cell/StructureBuilder.h"

#include "build_cell/AtomsDescription.h"
#include "build_cell/StructureDescription.h"
#include "build_cell/StructureDescriptionMap.h"
#include "common/Atom.h"
#include "common/Constants.h"
#include "common/Structure.h"

namespace sstbx {
namespace build_cell {

common::StructurePtr
StructureBuilder::buildStructure(const StructureDescription & description, DescriptionMapPtr & outDescriptionMap)
{
	// Create a new blank structure
  common::StructurePtr structure(new common::Structure());
  outDescriptionMap.reset(new StructureDescriptionMap(description, *structure.get()));

  myAtomsVolume = 0.0;

  myCurrentPair.first = structure.get();
  myCurrentPair.second = outDescriptionMap.get();

  description.traversePreorder(*this);

  // Reset the current structure pair
  myCurrentPair.first = NULL;
  myCurrentPair.second = NULL;

  return structure;
}

bool StructureBuilder::visitAtom(const AtomsDescription & atomDescription)
{
  const size_t numAtoms = atomDescription.getCount();
  ::boost::optional<double> radius;
  for(size_t i = 0; i < atomDescription.getCount(); ++i)
  {
    common::Atom & atom = myCurrentPair.first->newAtom(atomDescription.getSpecies());
    radius = atomDescription.getRadius();
    if(radius)
    {
      atom.setRadius(*radius);
    }
    myCurrentPair.second->insert(atomDescription.getParent(), &atomDescription, &atom);
  }

  // Try to determine the volume of the atoms
  radius = atomDescription.getRadius();
  if(radius)
  {
    const double radDouble = *radius;
    myAtomsVolume += numAtoms * 1.333333 * common::Constants::PI * radDouble * radDouble * radDouble;
  }

  return true;
}

double StructureBuilder::getAtomsVolume() const
{
  return myAtomsVolume;
}

}
}
