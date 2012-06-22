/*
 * AtomGroupDescription.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#include "build_cell/AtomGroupDescription.h"

#include <algorithm>

#include <boost/foreach.hpp>

// Local includes
#include "build_cell/AtomConstraintDescription.h"
#include "build_cell/AtomsDescription.h"

namespace sstbx
{
namespace build_cell
{

AtomGroupDescription::AtomGroupDescription(AtomGroupDescription * const parent):
myParent(parent)
{}

AtomGroupDescription::~AtomGroupDescription()
{}

void AtomGroupDescription::setParent(AtomGroupDescription * const parent)
{
	myParent = parent;
}

const AtomConstraintDescription *
AtomGroupDescription::getAtomConstraint(const ConstraintDescriptionId id) const
{
	const AtomConstraintDescription * constraint = 0;

	AtomCMap::const_iterator it = myAtomConstraints.find(id);
	if(it != myAtomConstraints.end())
	{
		constraint = it->second;
	}
	else if(myParent)
	{
		// Pass it up the chain
		constraint = myParent->getAtomConstraintInherits(id);
	}

	return constraint;
}

template <class CType>
CType * AtomGroupDescription::getAtomConstraint(const ConstraintDescriptionId id) const
{
	AtomConstraintDescription * constraint = 0;

	AtomCMap::const_iterator it = myAtomConstraints.find(id);
	if(it != myAtomConstraints.end())
	{
		constraint = it->second;
	}
	else if(myParent)
	{
		// Pass it up the chain
		constraint = myParent->getAtomConstraintInherits(id);
	}

	return static_cast<CType>(constraint);
}

void AtomGroupDescription::addAtomConstraint(AtomConstraintDescription * const atomConstraint)
{
  SSE_ASSERT(atomConstraint);

  myAtomConstraints.insert(atomConstraint->getType(), atomConstraint);
}

const AtomGroupDescription::GroupsContainer &
AtomGroupDescription::getChildGroups() const
{
	return myChildGroups;
}

bool AtomGroupDescription::removeAtomConstraint(const AtomConstraintDescription * const atomConstraint)
{
	AtomCMap::iterator it =	myAtomConstraints.find(atomConstraint->getType());

	if(it == myAtomConstraints.end() || (*it).second != atomConstraint) return false;

	myAtomConstraints.erase(it);

	return true;
}


void AtomGroupDescription::addChild(AtomGroupDescription * const group)
{
	// Add this child group to our vector of children
	myChildGroups.push_back(group);

	// Tell it that we are the new parent
	group->setParent(this);
}

void AtomGroupDescription::clearChildGroups()
{
  myChildGroups.clear();
}

const AtomGroupDescription::AtomsContainer &
AtomGroupDescription::getChildAtoms() const
{
	return myChildAtoms;
}

void AtomGroupDescription::addChild(AtomsDescription * const atoms)
{
	myChildAtoms.push_back(atoms);

	// Tell it that we are the new parent
	atoms->setParent(this);
}

void AtomGroupDescription::clearChildAtoms()
{
  myChildAtoms.clear();
}


const AtomConstraintDescription *
AtomGroupDescription::getAtomConstraintInherits(const ConstraintDescriptionId id) const
{
	const AtomConstraintDescription * constraint = 0;

	// Try to find the constraint amongst my constraints
	AtomCMap::const_iterator it = myAtomConstraints.find(id);
	if(it != myAtomConstraints.end() && it->second->inherits())
	{
		constraint = it->second;
	}
	else if(myParent)
	{
		// Pass it up the chain
		constraint = myParent->getAtomConstraintInherits(id);
	}

	return constraint;
}

}
}
