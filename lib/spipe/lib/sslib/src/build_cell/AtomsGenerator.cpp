/*
 * AtomsGenerator.h
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

AtomsGenerator::AtomsGenerator():
myNumReplicas(1),
myTransformMode(TransformMode::FIXED)
{
  myPos.zeros();

  myRot.zeros();
  myRot(2) = 1.0; // Need this so we don't get NaNs in calculating roation
}

AtomsGenerator::AtomsGenerator(const AtomsGenerator & toCopy):
AbsAtomsGenerator(toCopy),
myNumReplicas(toCopy.myNumReplicas),
myTransformMode(toCopy.myTransformMode),
myPos(toCopy.myPos),
myRot(toCopy.myRot)
{}

int AtomsGenerator::getNumReplicas() const
{
  return myNumReplicas;
}

void AtomsGenerator::setNumReplicas(const int numReplicas)
{
  if(myNumReplicas == numReplicas)
    return;

  myNumReplicas = numReplicas;
  invalidateTickets();
}

int AtomsGenerator::getTransformMode() const
{
  return myTransformMode;
}

void AtomsGenerator::setTransformMode(const int mode)
{
  myTransformMode = mode;
}

const ::arma::vec3 & AtomsGenerator::getPosition() const
{
  return myPos;
}

void AtomsGenerator::setPosition(const ::arma::vec3 & pos)
{
  myPos = pos;
}

const ::arma::vec4 & AtomsGenerator::getRotation() const
{
  return myRot;
}

void AtomsGenerator::setRotation(const ::arma::vec4 & rot)
{
  myRot = rot;
}

GenerationOutcome
AtomsGenerator::generateFragment(
  StructureBuild & build,
  const GenerationTicket ticket,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  GenerationOutcome outcome;
  for(size_t i = 0; i < myNumReplicas; ++i)
  {
    outcome = AbsAtomsGenerator::generateFragment(build, ticket, speciesDb);
    if(!outcome.isSuccess())
      return outcome;
  }
  return outcome;
}

StructureContents AtomsGenerator::getGenerationContents(
  const GenerationTicket ticket,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  StructureContents contents = AbsAtomsGenerator::getGenerationContents(ticket, speciesDb);
  contents *= myNumReplicas;
  return contents;
}

IFragmentGeneratorPtr AtomsGenerator::clone() const
{
  return IFragmentGeneratorPtr(new AtomsGenerator(*this));
}

::arma::mat44 AtomsGenerator::generateTransform(const StructureBuild & build) const
{
  using namespace utility::cart_coords_enum;

  ::arma::vec4 axisAngle = myRot;
  if(myTransformMode & TransformMode::RAND_ROT_DIR)
    axisAngle.rows(X, Z) = math::normaliseCopy(::arma::randu< ::arma::vec>(3));
  if(myTransformMode & TransformMode::RAND_ROT_ANGLE)
    axisAngle(3) = math::randu(0.0, common::constants::TWO_PI);

  ::arma::vec3 pos = myPos;
  if(myTransformMode & TransformMode::RAND_POS)
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
