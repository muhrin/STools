/*
 * AtomsDescription.h
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */


// INCLUDES /////////////////
#include "build_cell/AtomsDescription.h"

#include "build_cell/AtomConstraintDescription.h"

#include "common/AtomSpeciesId.h"

namespace sstbx {
namespace build_cell {

AtomsDescription::AtomsDescription():
mySpecies(sstbx::common::AtomSpeciesId::DUMMY),
myCount(1)
{
}

AtomsDescription::AtomsDescription(const ::sstbx::common::AtomSpeciesId::Value  species, const size_t count):
mySpecies(species),
myCount(count)
{}
//
//const AtomConstraintDescription *
//AtomsDescription::getAtomConstraint(const ConstraintDescriptionId id) const
//{
//	AtomCMap::const_iterator it = myAtomConstraints.find(id);
//	if(it == myAtomConstraints.end())
//		return NULL;
//
//	return it->second;
//}
//
//void AtomsDescription::addAtomConstraint(AtomConstraintDescription * const atomConstraint)
//{
//	myAtomConstraints.insert(atomConstraint->getType(), atomConstraint);
//}
//
//bool AtomsDescription::removeAtomConstraint(const AtomConstraintDescription * const atomConstraint)
//{
//	AtomCMap::iterator it =	myAtomConstraints.find(atomConstraint->getType());
//
//	if(it == myAtomConstraints.end() || (*it).second != atomConstraint) return false;
//
//	myAtomConstraints.erase(it);
//
//	return true;
//}

const ::sstbx::common::AtomSpeciesId::Value & AtomsDescription::getSpecies() const
{
	return mySpecies;
}

void AtomsDescription::setSpecies(const ::sstbx::common::AtomSpeciesId::Value  species)
{
	mySpecies = species;
}

size_t AtomsDescription::getCount() const
{
	return myCount;
}

void AtomsDescription::setCount(const size_t newCount)
{
	myCount = newCount;
}

const OptionalDouble & AtomsDescription::getRadius() const
{
  return myRadius;
}

void AtomsDescription::setRadius(const OptionalDouble radius)
{
  myRadius = radius;
}

const OptionalVec3 & AtomsDescription::getPosition() const
{
  return myPosition;
}

void AtomsDescription::setPosition(const OptionalVec3 & pos)
{
  myPosition = pos;
}

}
}
