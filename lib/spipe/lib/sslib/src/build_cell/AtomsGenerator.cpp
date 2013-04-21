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
#include "common/Constants.h"
#include "common/Structure.h"
#include "math/Matrix.h"
#include "math/Random.h"
#include "utility/IndexingEnums.h"
#include "utility/SharedHandle.h"

namespace sstbx {
namespace build_cell {

AtomsGenerator::AtomsGenerator(AtomsGeneratorConstructionInfo & constructionInfo):
myNumReplicas(constructionInfo.numReplicas),
myGenShape(constructionInfo.genShape.release())
{
  myTransformMask = constructionInfo.transformMask;

  if(constructionInfo.pos)
    myTranslation = *constructionInfo.pos;
  else
    myTranslation.zeros();

  if(constructionInfo.rot)
    myRotation = *constructionInfo.rot;
  else
  {
    myRotation.zeros();
    myRotation(2) = 1.0; // Need this so we don't get NaNs in calculating roation
  }
}

AtomsGenerator::AtomsGenerator(const AtomsGenerator & toCopy):
myNumReplicas(toCopy.myNumReplicas),
myAtoms(toCopy.myAtoms),
myGenShape(toCopy.myGenShape->clone()),
myTranslation(toCopy.myTranslation),
myRotation(toCopy.myRotation),
myTransformMask(toCopy.myTransformMask)
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

GenerationOutcome
AtomsGenerator::generateFragment(
  StructureBuild & build,
  const GenerationTicket ticket,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  typedef ::std::vector<unsigned int> Multiplicities;

  // Get the ticket
  TicketsMap::const_iterator it = myTickets.find(ticket.getId());
  SSLIB_ASSERT_MSG(it != myTickets.end(), "Asked to build structure with ticket we don't recognise.");
  const AtomCounts & counts = it->second.atomCounts;
  if(counts.empty())
    return GenerationOutcome::success();

  GenerationOutcome outcome;

  common::Structure & structure = build.getStructure();

  // Do we have any symmetry?
  const bool usingSymmetry = build.getSymmetryGroup() != NULL;

  AtomPosition position;
  Multiplicities multiplicities;
  Multiplicities possibleMultiplicities;
  if(usingSymmetry)
  {
    possibleMultiplicities = build.getSymmetryGroup()->getMultiplicities();
    possibleMultiplicities.push_back(build.getSymmetryGroup()->numOps());
  }
  else
    possibleMultiplicities.push_back(1); // No symmetry so all points have multiplicity 1

  ::arma::mat44 transform;
  for(size_t i = 0; i < myNumReplicas; ++i)
  {
    transform = generateTransform(build);

    BOOST_FOREACH(AtomCounts::const_reference atomsDesc, counts)
    {
      if(atomsDesc.first->getPosition())
      {
        // If the atom has a fixed position then we should not apply symmetry
        // and the multiplicity should be 1
        // TODO: Check if this position is compatible with our symmetry operators!
        multiplicities.insert(multiplicities.begin(), atomsDesc.second, 1);
      }
      else
        multiplicities = symmetry::generateMultiplicities(atomsDesc.second, possibleMultiplicities);

      if(multiplicities.empty())
      {
        outcome.setFailure("Couldn't factor atom multiplicities into number of sym ops.");
        return outcome;
      }

      // Go over the multiplicities inserting the atoms
      BOOST_FOREACH(const unsigned int multiplicity, multiplicities)
      {
        common::Atom & atom = structure.newAtom(atomsDesc.first->getSpecies());
        BuildAtomInfo & info = build.createAtomInfo(atom);
        info.setMultiplicity(multiplicity);

        position = generatePosition(info, *atomsDesc.first, build, multiplicity, transform);
        atom.setPosition(position.first);
        info.setFixed(position.second);

        atom.setRadius(getRadius(*atomsDesc.first, speciesDb));
      }
      multiplicities.clear(); // Reset for next loop
    }
  }

  // Finally solve any atom overlap
  if(build.extrudeAtoms())
    outcome.setSuccess();
  else
    outcome.setFailure("Failed to extrude atoms, maybe the volume is too small.");

  return outcome;
}

AtomsGenerator::GenerationTicket AtomsGenerator::getTicket()
{
  GenerationTicket::IdType ticketId = ++myLastTicketId;

  // Generate a random number of atoms
  AtomsDescription::CountRange count;
  AtomCounts counts;
  BOOST_FOREACH(const AtomsDescription & atomsDesc, myAtoms)
  {
    count = atomsDesc.getCount();
    if(count.nullSpan())
      counts[&atomsDesc] = count.lower();
    else
      counts[&atomsDesc] = math::randu(count.lower(), count.upper());
  }
  myTickets[ticketId].atomCounts = counts;
  return GenerationTicket(ticketId);
}

StructureContents AtomsGenerator::getGenerationContents(
  const GenerationTicket ticket,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  StructureContents contents;

  // Get the ticket
  TicketsMap::const_iterator it = myTickets.find(ticket.getId());
  SSLIB_ASSERT_MSG(it != myTickets.end(), "Asked to build structure with ticket we don't recognise.");
  const AtomCounts & counts = it->second.atomCounts;

  double radius;
  BOOST_FOREACH(AtomCounts::const_reference atomsDesc, counts)
  {
    radius = getRadius(*atomsDesc.first, speciesDb);
    contents.addAtoms(static_cast<size_t>(atomsDesc.second * myNumReplicas), radius);
  }

  return contents;
}

void AtomsGenerator::handleReleased(const GenerationTicketId & id)
{
  // Get the ticket
  TicketsMap::iterator it = myTickets.find(id);
  SSLIB_ASSERT_MSG(it != myTickets.end(), "Being notified of ticket release for a ticket id we don't recognise.");
  myTickets.erase(it);
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
  const unsigned int multiplicity,
  const ::arma::mat44 & transformation
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
    position.first = math::transformCopy(*optionalPosition, transformation);
    position.second = true;
  }
  else
  {
    position.first = getGenShape(build).randomPoint(&transformation);

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
        const SymmetryGroup::EigenspacesAndMasks * const spaces =
          build.getSymmetryGroup()->getEigenspacesAndMasks(multiplicity);

        ::arma::vec3 newPos;
        SymmetryGroup::OpMask opMask;
        if(!generateSpecialPosition(newPos, opMask, *spaces, getGenShape(build), transformation))
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
  const SymmetryGroup::EigenspacesAndMasks & spaces,
  const IGeneratorShape & genShape,
  const ::arma::mat44 & transformation
) const
{
  typedef ::std::vector<size_t> Indices;
  // Generate a list of the indices
  Indices indices;
  for(size_t i = 0; i < spaces.size(); ++i)
    indices.push_back(i);
  
  Indices::iterator it;
  OptionalArmaVec3 pos;
  while(!indices.empty())
  {
    // Get a random one in the list
    it = indices.begin() + math::randu<size_t>(indices.size());
    pos = generateSpeciesPosition(spaces[*it].first, genShape, transformation);
    if(pos)
    {
      // Copy over the position and mask
      posOut = *pos;
      opMaskOut = spaces[*it].second;
      return true;
    }
    indices.erase(it);
  }
  return false; // Couldn't find one
}

OptionalArmaVec3 AtomsGenerator::generateSpeciesPosition(
  const SymmetryGroup::Eigenspace & space,
  const IGeneratorShape & genShape,
  const ::arma::mat44 & transformation
) const
{
  // Select the correct function depending on the number of eigenvectors
  if(space.n_cols == 1)
    return genShape.randomPointOnAxis(space, &transformation);
  else if(space.n_cols == 2)
    return genShape.randomPointInPlane(space.col(0), space.col(1), &transformation);
  return OptionalArmaVec3(::arma::zeros< ::arma::vec>(3));
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

const IGeneratorShape & AtomsGenerator::getGenShape(const StructureBuild & build) const
{
  if(myGenShape.get())
    return *myGenShape;
  else
    return build.getGenShape();
}

::arma::mat44 AtomsGenerator::generateTransform(const StructureBuild & build) const
{
  using namespace utility::cart_coords_enum;

  ::arma::vec4 axisAngle = myRotation;
  if(myTransformMask & TransformSettings::RAND_ROT_DIR)
    axisAngle.rows(X, Z) = math::normaliseCopy(::arma::randu< ::arma::vec>(3));
  if(myTransformMask & TransformSettings::RAND_ROT_ANGLE)
    axisAngle(3) = math::randu(0.0, common::constants::TWO_PI);

  ::arma::vec3 pos = myTranslation;
  if(myTransformMask & TransformSettings::RAND_POS)
  {
    // Create a random point somewhere within the global generation shape
    pos = build.getGenShape().randomPoint();
  }

  ::arma::mat44 transform;
  transform.eye();
  math::setTranslation(transform, pos);
  math::setRotation(transform, axisAngle);

  return transform;
}

}
}
