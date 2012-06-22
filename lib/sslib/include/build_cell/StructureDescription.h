/*
 * StructureDescription.h
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#ifndef STRUCTURE_DESCRIPTION_H
#define STRUCTURE_DESCRIPTION_H

// INCLUDES ///////////////////

#include <boost/ptr_container/ptr_vector.hpp>

#include "build_cell/AtomGroupDescription.h"
#include "build_cell/StructureConstraintDescription.h"

// FORWARD DECLARES ///////////

namespace sstbx { namespace build_cell {

class StructureDescription : public AtomGroupDescription
{
public:
  typedef boost::ptr_vector<StructureConstraintDescription> ConstraintsContainer;

	virtual ~StructureDescription();

	void addStructureConstraint(StructureConstraintDescription * const structureConstraint);	

	ConstraintsContainer const & getStructureConstraints();

protected:


  ConstraintsContainer    myStructureConstraints;

};

}}

#endif /* STRUCTURE_DESCRIPTION_H */
