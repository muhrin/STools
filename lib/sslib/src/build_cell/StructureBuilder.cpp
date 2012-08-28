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

StructureBuilder::StructurePair
StructureBuilder::buildStructure(const StructureDescription & description)
{
	// Create a new blank structure
  common::StructurePtr structure(new common::Structure());
  DescriptionMapPtr descriptionMap(new StructureDescriptionMap(description, *structure.get()));

  myAtomsVolume = 0.0;

  StructurePair pair(structure, descriptionMap);
  myCurrentPair = &pair;

  description.traversePreorder(*this);

  return pair;
}

bool StructureBuilder::visitAtom(const AtomsDescription & atomDescription)
{
  common::Atom * atom;
  const size_t numAtoms = atomDescription.getCount();
  ::boost::optional<double> radius;
  for(size_t i = 0; i < atomDescription.getCount(); ++i)
  {
    radius = atomDescription.getRadius();
    if(radius)
    {
      atom = new common::Atom(atomDescription.getSpecies(), *radius);
    }
    else
    {
      atom = new common::Atom(atomDescription.getSpecies());
    }
    myCurrentPair->first->insertAtom(atom);
    myCurrentPair->second->insert(atomDescription.getParent(), &atomDescription, atom);
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
