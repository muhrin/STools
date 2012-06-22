/*
 * StructureDescription.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#include "build_cell/StructureDescription.h"

#include "build_cell/AtomConstraintDescription.h"
#include "build_cell/AtomsDescription.h"
#include "build_cell/StructureConstraintDescription.h"

namespace sstbx
{
namespace build_cell
{

StructureDescription::~StructureDescription()
{}

void StructureDescription::addStructureConstraint(
  StructureConstraintDescription * const structureConstraint)
{
	myStructureConstraints.push_back(structureConstraint);
}

const StructureDescription::ConstraintsContainer &
StructureDescription::getStructureConstraints()
{
	return myStructureConstraints;
}

}
}
