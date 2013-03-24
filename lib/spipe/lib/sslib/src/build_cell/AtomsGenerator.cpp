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
  Multiplicities possibleMultiplicities;
  if(usingSymmetry)
    possibleMultiplicities = build.getSymmetryGroup()->getMultiplicities();
  else
    possibleMultiplicities.push_back(1); // No symmetry so all points have multiplicity 1
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
  // Default is not fixed
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

        ::arma::vec3 newPos;
        SymmetryGroup::OpMask opMask;
        if(!generateSpecialPosition(newPos, opMask, *eigenVecOpsList))
        {
          // TODO: return error
          return position;
        }
        
        // Save the position
        position.first = newPos;
        atomInfo.setOperatorsMask(opMask);
        position.second = true; // Fix the position so it doesn't get moved when extruding atoms
      }
    }
  }

  return position;
}

bool AtomsGenerator::generateSpecialPosition(
  ::arma::vec3 & posOut,
  SymmetryGroup::OpMask & opMaskOut,
  const SymmetryGroup::EigenvectorsOpsList & eigenvecLists) const
{
  typedef ::std::vector<size_t> Indices;
  // Generate a list of the indices
  Indices indices;
  for(size_t i = 0; i < eigenvecLists.size(); ++i)
    indices.push_back(i);
  
  Indices::iterator it;
  OptionalArmaVec3 pos;
  while(!indices.empty())
  {
    // Get a random one in the list
    it = indices.begin() + math::randu<size_t>(indices.size() - 1);
    pos = generateSpeciesPosition(eigenvecLists[*it].first);
    if(pos)
    {
      // Copy over the position and mask
      posOut = *pos;
      opMaskOut = eigenvecLists[*it].second;
      return true;
    }
    indices.erase(it);
  }
  return false; // Couldn't find one
}

OptionalArmaVec3 AtomsGenerator::generateSpeciesPosition(
  const SymmetryGroup::EigenvectorsList & eigenvecs) const
{
  if(eigenvecs.empty())
    return ::arma::zeros< ::arma::vec>(3);

  // Select the correct function depending on the number of eigenvectors
  if(eigenvecs.size() == 1)
    return myGenShape->randomPointOnAxis(eigenvecs[0]);
  else if(eigenvecs.size() == 2)
    return myGenShape->randomPointInPlane(eigenvecs[0], eigenvecs[1]);
  else
  {
    // Project the generated point onto the invariant eigenvector
    ::arma::vec3 oldPos = myGenShape->randomPoint();
    ::arma::vec3 pos;
    pos.zeros();
    for(size_t i = 0; i < eigenvecs.size(); ++i)
    {
      pos += ::arma::dot(oldPos, eigenvecs[i]) * eigenvecs[i];
    }
    return pos;
  }
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
