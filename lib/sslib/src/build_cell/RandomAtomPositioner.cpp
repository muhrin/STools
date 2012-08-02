/*
 * RandomAtomPositioner.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////////
#include "build_cell/RandomAtomPositioner.h"

#include <vector>

#include <boost/foreach.hpp>

#include <armadillo>

#include "build_cell/AtomGroupDescription.h"
#include "build_cell/AtomsDescription.h"
#include "build_cell/StructureDescriptionMap.h"
#include "common/AbstractFmidCell.h"
#include "common/Atom.h"
#include "common/Structure.h"
#include "common/Types.h"

namespace sstbx {
namespace build_cell {

RandomAtomPositioner::RandomAtomPositioner(StructureDescriptionMap & descriptionMap):
myDescriptionMap(descriptionMap),
myStructure(descriptionMap.getStructure())
{}

void RandomAtomPositioner::enteringAtomGroup(const AtomGroupDescription & atomGroup)
{
  const common::AbstractFmidCell * const cell = myStructure.getUnitCell();
  pushDisplacement(cell->randomPoint());
}

void RandomAtomPositioner::leavingAtomGroup(const AtomGroupDescription & atomGroup)
{
  popDisplacement();
}

bool RandomAtomPositioner::visitAtomGroup(const AtomGroupDescription & atomGroup)
{
  const common::AbstractFmidCell * const cell = myStructure.getUnitCell();

  // Place the atoms in this group
  common::Atom * atom;
  ::arma::vec3 pos;
  const StructureDescriptionMap::GroupAtomsRange range = myDescriptionMap.getAtoms(&atomGroup);
  
  BOOST_FOREACH(atom, range)
  {
    pos = cell->wrapVec(myCurrentPosition + cell->randomPoint());
    atom->setPosition(cell->randomPoint());
  }

  return true;
}

void RandomAtomPositioner::pushDisplacement(const arma::vec3 &displacement)
{
  myCurrentPosition += displacement;
  myDisplacementsStack.push_back(displacement);
}

void RandomAtomPositioner::popDisplacement()
{
  const ::arma::vec3 & last = myDisplacementsStack.back();
  myCurrentPosition -= last;
  // Remove the last element
  myDisplacementsStack.pop_back();
}

}
}
