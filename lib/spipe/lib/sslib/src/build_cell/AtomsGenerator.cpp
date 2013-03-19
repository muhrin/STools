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
#include "build_cell/SymmetryFunctions.h"
#include "build_cell/SymmetryGroup.h"
#include "common/Atom.h"
#include "common/AtomSpeciesDatabase.h"
#include "common/Structure.h"
#include "common/Utils.h"
#include "math/Random.h"
#include "utility/SharedHandle.h"

namespace sstbx {
namespace build_cell {

AtomsGenerator::AtomsGenerator(const AtomsGenerator & toCopy):
myAtoms(toCopy.myAtoms),
myGenShape(toCopy.myGenShape->clone())
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

const IGeneratorShape * AtomsGenerator::getGeneratorShape() const
{
  return myGenShape.get();
}

void AtomsGenerator::setGeneratorShape(GenShapePtr shape)
{
  myGenShape = shape;
}

GenerationOutcome
AtomsGenerator::generateFragment(
  StructureBuild & build,
  const GenerationTicket ticket,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  typedef ::std::vector<unsigned int> Multiplicities;

  GenerationOutcome outcome;

  common::Structure & structure = build.getStructure();

  // Do we have any symmetry?
  const bool usingSymmetry = build.getSymmetryGroup() != NULL;

  // First insert all the atoms into the structure
  AtomPosition position;
  Multiplicities multiplicities;
  const Multiplicities possibleMultiplicities(build.getSymmetryGroup()->getMultiplicities());
  BOOST_FOREACH(const AtomsDescription & atomsDesc, myAtoms)
  {
    if(atomsDesc.getPosition())
    {
      // If the atom has a fixed position then we should not apply symmetry
      // and the multiplicity should be 1
      // TODO: Check if this position is compatible with our symmetry operators!
      multiplicities.insert(multiplicities.begin(), atomsDesc.getCount(), 1);
    }
    else
    {
      multiplicities = symmetry::generateMultiplicities(atomsDesc.getCount(), possibleMultiplicities);
    }

    if(multiplicities.empty())
    {
      outcome.setFailure("Couldn't factor atom multiplicities into number of sym ops.");
      return outcome;
    }

    // Go over the multiplicities inserting the atoms
    BOOST_FOREACH(const unsigned int multiplicity, multiplicities)
    {
      common::Atom & atom = structure.newAtom(atomsDesc.getSpecies());
      BuildAtomInfo & info = build.createAtomInfo(atom);
      info.setMultiplicity(multiplicity);

      position = generatePosition(info, atomsDesc, build, multiplicity);
      atom.setPosition(position.first);
      info.setFixed(position.second);

      atom.setRadius(getRadius(atomsDesc, speciesDb));
    }
    multiplicities.clear(); // Reset for next loop
  }

  // Finally solve any atom overlap
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
AtomsGenerator::generatePosition(
  BuildAtomInfo & atomInfo,
  const AtomsDescription & atom,
  const StructureBuild & build,
  const unsigned int multiplicity
) const
{
  SSLIB_ASSERT_MSG(
    !(multiplicity > 1 && !build.getSymmetryGroup()),
    "If we have a multiplicity of more than one then there must be a symmetry group"
  );

  const bool usingSymmetry = build.getSymmetryGroup() != NULL;

  AtomPosition position;
  // Default is not fixex
  position.second = false;

  OptionalArmaVec3 optionalPosition;

  optionalPosition = atom.getPosition();
  if(optionalPosition)
  {
    // Position is fixed
    position.first = *optionalPosition;
    position.second = true;
  }
  else
  {
    if(myGenShape.get())
      position.first = myGenShape->randomPoint();
    else
      position.first = build.getRandomPoint();

    // Do we need to apply symmetry and does the atom need to be on a 'special' position
    if(usingSymmetry)
    {
      if(multiplicity == build.getSymmetryGroup()->numOps())
      {
        // Apply all operators (true for entire op mask)
        atomInfo.setOperatorsMask(::std::vector<bool>(build.getSymmetryGroup()->numOps(), true));
      }
      else // Special position
      {
        const SymmetryGroup::EigenvectorsOpsList * const eigenVecOpsList =
          build.getSymmetryGroup()->getEigenvectorsOpsList(multiplicity);

        // Choose one of them
        const size_t invariant = math::rand<size_t>(eigenVecOpsList->size());
        // Project the generated point onto the invariant eigenvector(s)
        ::arma::vec3 newPos;
        newPos.zeros();
        for(size_t i = 0; i < (*eigenVecOpsList)[invariant].first.size(); ++i)
        {
          const ::arma::vec & eigenVec = (*eigenVecOpsList)[invariant].first[i];
          newPos += ::arma::dot(position.first, eigenVec) * eigenVec;
        }
        position.first = newPos;

        atomInfo.setOperatorsMask((*eigenVecOpsList)[invariant].second);
        position.second = true; // Fix the position so it doesn't get moved when extruding atoms
      }
    }
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
