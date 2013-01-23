/*
 * StructureBuilder.h
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */


// INCLUDES /////////////////
#include "build_cell/AtomsGenerator.h"

#include <boost/foreach.hpp>

#include "build_cell/AtomsDescription.h"
#include "build_cell/GenerationOutcome.h"
#include "build_cell/StructureBuild.h"
#include "build_cell/StructureContents.h"
#include "common/Atom.h"
#include "common/AtomSpeciesDatabase.h"
#include "common/Structure.h"
#include "common/Utils.h"
#include "utility/SharedHandle.h"

namespace sstbx {
namespace build_cell {

AtomsGenerator::AtomsGenerator(const AtomsGenerator & toCopy):
myAtoms(toCopy.myAtoms),
myGenerationSphere(toCopy.myGenerationSphere)
{}

size_t AtomsGenerator::numAtoms() const
{
  return myAtoms.size();
}

AtomsGenerator::iterator AtomsGenerator::beginAtoms()
{
  return myAtoms.begin();
}

AtomsGenerator::const_iterator AtomsGenerator::beginAtoms() const
{
  return myAtoms.begin();
}

AtomsGenerator::iterator AtomsGenerator::endAtoms()
{
  return myAtoms.end();
}

AtomsGenerator::const_iterator AtomsGenerator::endAtoms() const
{
  return myAtoms.end();
}

AtomsGenerator::iterator AtomsGenerator::addAtoms(const AtomsDescription & atoms)
{
  return myAtoms.insert(myAtoms.end(), atoms);
}

void AtomsGenerator::eraseAtoms(AtomsGenerator::iterator pos)
{
  myAtoms.erase(pos);
}

const AtomsGenerator::OptionalSphere & AtomsGenerator::getGenerationSphere() const
{
  return myGenerationSphere;
}

void AtomsGenerator::setGenerationSphere(const OptionalSphere & sphere)
{
  myGenerationSphere = sphere;
}

GenerationOutcome
AtomsGenerator::generateFragment(
  StructureBuild & build,
  const GenerationTicket ticket,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  GenerationOutcome outcome;

  common::Structure & structure = build.getStructure();
  // First insert all the atoms into the structure
  AtomPosition position;
  BOOST_FOREACH(const AtomsDescription & atomsDesc, myAtoms)
  {
    for(unsigned int i = 0; i < atomsDesc.getCount(); ++i)
    {
      common::Atom & atom = structure.newAtom(atomsDesc.getSpecies());
      BuildAtomInfo info(atom);

      position = generatePosition(atomsDesc, build);

      atom.setPosition(position.first);
      info.setFixed(position.second);

      atom.setRadius(getRadius(atomsDesc, speciesDb));

      // Tell the structure build about this atom
      build.insertAtomInfo(info);
    }
  }

  if(build.extrudeAtoms())
    outcome.setSuccess();
  else
    outcome.setFailure("Failed to extrude atoms, maybe the volume is too small.");

  return outcome;
}

AtomsGenerator::GenerationTicket AtomsGenerator::getTicket() const
{
  return GenerationTicket(0);
}

StructureContents AtomsGenerator::getGenerationContents(
  const GenerationTicket ticket,
  const common::AtomSpeciesDatabase & speciesDb) const
{
  StructureContents contents;

  double radius;
  BOOST_FOREACH(const AtomsDescription & atoms, myAtoms)
  {
    radius = getRadius(atoms, speciesDb);
    contents.addAtoms(atoms.getCount(), radius);
  }

  return contents;
}

void AtomsGenerator::handleReleased(const GenerationTicketId & id)
{
  // Nothing to do: ticket doesn't identify a particular generation instance
}

IFragmentGeneratorPtr AtomsGenerator::clone() const
{
  return IFragmentGeneratorPtr(new AtomsGenerator(*this));
}

AtomsGenerator::AtomPosition
AtomsGenerator::generatePosition(const AtomsDescription & atom, const StructureBuild & build) const
{
  AtomPosition position;
  // Default is not fixex
  position.second = false;

  OptionalVec3 optionalPosition;

  optionalPosition = atom.getPosition();
  if(optionalPosition)
  {
    // Position is fixed
    position.first = *optionalPosition;
    position.second = true;
  }
  else
  {
    if(myGenerationSphere)
      position.first = myGenerationSphere->randomPoint();
    else
      position.first = build.getRandomPoint();
  }

  return position;
}

double AtomsGenerator::getRadius(
  const AtomsDescription & atom,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  double radius;
  ::boost::optional<double> optionalRadius;

  // Try getting it from the atom
  optionalRadius = atom.getRadius();
  if(optionalRadius)
    radius = *optionalRadius;
  else
  {
    // Then try from the database
    optionalRadius = speciesDb.getRadius(atom.getSpecies());
    if(optionalRadius)
      radius = *optionalRadius;
    else
      radius = 0.0;
  }

  return radius;
}

}
}
