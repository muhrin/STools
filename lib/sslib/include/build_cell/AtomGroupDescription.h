/*
 * AtomGroupDescription.h
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#ifndef ATOM_GROUP_DESCRIPTION_H
#define ATOM_GROUP_DESCRIPTION_H

// INCLUDES ///////////////////
#include "SSLib.h"

#include <boost/ptr_container/ptr_map.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include "build_cell/AtomConstraintDescription.h"
#include "build_cell/IAtomConstrainable.h"
#include "build_cell/ConstraintDescriptionId.h"



// FORWARD DECLARES ///////////
namespace sstbx
{
namespace build_cell
{
class AtomsDescription;
class IConstraintDescription;
}
}


namespace sstbx { namespace build_cell {

class AtomGroupDescription : public IAtomConstrainable
{
public:

  typedef ::boost::ptr_vector<AtomsDescription>     AtomsContainer;
  typedef ::boost::ptr_vector<AtomGroupDescription> GroupsContainer;

	friend class AtomsDescription;

	AtomGroupDescription(AtomGroupDescription * const parent = NULL);
	virtual ~AtomGroupDescription();

	/** Child groups */

	const GroupsContainer & getChildGroups() const;

	void addChild(AtomGroupDescription * const childGroup);

  void clearChildGroups();

	/** Child atoms */

	const AtomsContainer & getChildAtoms() const;

	void addChild(AtomsDescription * const childAtom);

  void clearChildAtoms();

	// From IAtomConstrainable //
	virtual const AtomConstraintDescription * getAtomConstraint(const ConstraintDescriptionId id) const;
	virtual void addAtomConstraint(AtomConstraintDescription * const atomConstraint);
	virtual bool removeAtomConstraint(const AtomConstraintDescription * const atomConstraint);
  // End from IAtomConstrainable

	template <class CType>
	CType * getAtomConstraint(const ConstraintDescriptionId id) const;

protected:

  typedef ::boost::ptr_map<const ConstraintDescriptionId, AtomConstraintDescription> AtomCMap;

	/** Parent group */

	void setParent(AtomGroupDescription * const parent);

	const AtomConstraintDescription *
    getAtomConstraintInherits(const ConstraintDescriptionId id) const;

	AtomGroupDescription *					myParent;

	AtomsContainer		              myChildAtoms;
	GroupsContainer                 myChildGroups;

	AtomCMap myAtomConstraints;

};

}}

#endif /* ATOM_GROUP_DESCRIPTION_H */
