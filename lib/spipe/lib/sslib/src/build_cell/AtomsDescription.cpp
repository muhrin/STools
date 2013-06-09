/*
 * AtomsDescription.h
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */


// INCLUDES /////////////////
#include "build_cell/AtomsDescription.h"

#include "SSLibAssert.h"
#include "common/AtomSpeciesId.h"

namespace sstbx {
namespace build_cell {

AtomsDescription::AtomsDescription():
myCount(1)
{}

AtomsDescription::AtomsDescription(const ::sstbx::common::AtomSpeciesId::Value  species, const size_t count):
mySpecies(species),
myCount(count)
{}

AtomsDescription::AtomsDescription(const ::sstbx::common::AtomSpeciesId::Value  species, const CountRange count):
mySpecies(species),
myCount(count)
{}

const ::sstbx::common::AtomSpeciesId::Value & AtomsDescription::getSpecies() const
{
	return mySpecies;
}

void AtomsDescription::setSpecies(const ::sstbx::common::AtomSpeciesId::Value  species)
{
	mySpecies = species;
}

AtomsDescription::CountRange AtomsDescription::getCount() const
{
	return myCount;
}

void AtomsDescription::setCount(const int count)
{
  SSLIB_ASSERT(count > 0);

  myCount.set(count, count);
}

void AtomsDescription::setCount(const CountRange count)
{
  SSLIB_ASSERT(count.lower() >= 0);

  myCount = count;
}

const OptionalDouble & AtomsDescription::getRadius() const
{
  return myRadius;
}

void AtomsDescription::setRadius(const OptionalDouble radius)
{
  myRadius = radius;
}

const OptionalArmaVec3 & AtomsDescription::getPosition() const
{
  return myPosition;
}

void AtomsDescription::setPosition(const OptionalArmaVec3 & pos)
{
  myPosition = pos;
}

}
}
