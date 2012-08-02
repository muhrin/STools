/*
 * Structure.cpp
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES /////////////////////////////////////
#include "common/Structure.h"

#include <vector>

#include "common/AbstractFmidCell.h"
#include "common/Atom.h"
#include "common/StructureTreeEvent.h"
#include "common/Types.h"

namespace sstbx {
namespace common {

Structure::Structure(AbstractFmidCell * const cell):
myCell(cell),
myNumAtomsDescendent(0),
myAtomPositionsCurrent(false)
{
}

const std::string & Structure::getName() const
{
	return myName;
}

void Structure::setName(const std::string & name)
{
	myName = name;
}

AbstractFmidCell * Structure::getUnitCell()
{
	return myCell.get();
}

const AbstractFmidCell * Structure::getUnitCell() const
{
	return myCell.get();
}

void Structure::setUnitCell(const UnitCellPtr cell)
{
	myCell = cell;
}

size_t Structure::getNumAtomsDescendent() const
{
	return myNumAtomsDescendent;
}

void Structure::getAtomPositionsDescendent(Mat & posMtx, const size_t i) const
{
	// Do we need to update the buffer?
	if(!myAtomPositionsCurrent)
	{
		myAtomPositionsBuffer.reset();
		AtomGroup::getAtomPositionsDescendent(myAtomPositionsBuffer);
		myAtomPositionsCurrent = true;
	}

	// TODO: FIX THIS, THIS WON'T WORK FOR ATOM GROUPS
	// Insert the columns at the desired location
	//posMtx.insert_cols(i, myAtomPositionsBuffer);
	posMtx = myAtomPositionsBuffer;
}

void Structure::setAtomPositionsDescendent(const Mat & posMtx)
{
	// Set the atom positions
	AtomGroup::setAtomPositionsDescendent(posMtx);
	// Save the new positions in the buffer
	myAtomPositionsBuffer	= posMtx;
	myAtomPositionsCurrent	= true;
}

void Structure::eventFired(const StructureTreeEvent & evt)
{
	if(evt.getEventType() == StructureTreeEvent::GROUP_INSERTED)
	{
		myAtomPositionsCurrent = false;
		childAdded(evt.getGroup());
	}
	else if(evt.getEventType() == StructureTreeEvent::GROUP_REMOVED)
	{
		myAtomPositionsCurrent = false;
		childRemoved(evt.getGroup());
	}
	else if(evt.getEventType() == StructureTreeEvent::ATOM_INSERTED)
	{
		myAtomPositionsCurrent = false;
		atomAdded(evt.getAtom());
	}
	else if(evt.getEventType() == StructureTreeEvent::ATOM_REMOVED)
	{
		myAtomPositionsCurrent = false;
		atomRemoved(evt.getAtom());
	}
	else if(evt.getEventType() == StructureTreeEvent::ATOM_CHANGED)
	{
		myAtomPositionsCurrent = false;
	}
}

void Structure::childAdded(AtomGroup & childGroup)
{
	using ::std::vector;

	// First go through adding all groups
	const vector<AtomGroup *> & subGroups = childGroup.getGroups();
	for(vector<AtomGroup * >::const_iterator it = subGroups.begin(), end = subGroups.end();
		it != end; ++it)
	{
		childAdded(**it);
	}

	// Now go through all atoms
	const vector<AtomPtr> & atoms = childGroup.getAtoms();
	for(vector<AtomPtr>::const_iterator it = atoms.begin(), end = atoms.end();
		it != end; ++it)
	{
		atomAdded(**it);
	}
}

void Structure::childRemoved(AtomGroup & childGroup)
{
	using ::std::vector;

	// First go through adding all groups
	const vector<AtomGroup *> & subGroups = childGroup.getGroups();
	for(vector<AtomGroup * >::const_iterator it = subGroups.begin(), end = subGroups.end();
		it != end; ++it)
	{
		childRemoved(**it);
	}

	// Now go through all atoms
	const vector<AtomPtr> & atoms = childGroup.getAtoms();
	for(vector<AtomPtr>::const_iterator it = atoms.begin(), end = atoms.end();
		it != end; ++it)
	{
		atomRemoved(**it);
	}
}

void Structure::atomAdded(Atom & atom)
{
	++myNumAtomsDescendent;
}

void Structure::atomRemoved(Atom & atom)
{
	--myNumAtomsDescendent;

	SSE_ASSERT(myNumAtomsDescendent > 0);
}

}}
