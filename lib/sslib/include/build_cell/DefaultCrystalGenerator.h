/*
 * DefaultCrystalGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef DEFAULT_CRYSTAL_GENERATOR_H
#define DEFAULT_CRYSTAL_GENERATOR_H

// INCLUDES /////////////////////////////////////////////////////
#include "ICrystalStructureGenerator.h"

// FORWARD DECLARES /////////////////////////////////////////////
namespace sstbx
{
namespace common
{
class AbstractFmidCell;
class AtomGroup;
class Structure;
}
namespace build_cell {

class AbstractConstraintDescription;
class AtomGroupDescription;
class ICellGenerator;
class RandomCellDescription;
class StructureBuilder;
class StructureDescription;
class StructureDescriptionMap;
}
}

namespace sstbx
{
namespace build_cell
{

class DefaultCrystalGenerator : public ICrystalStructureGenerator
{
public:

  typedef ICrystalStructureGenerator::Result Result;

	DefaultCrystalGenerator(const ICellGenerator &	cellGenerator);
  virtual ~DefaultCrystalGenerator() {}

	/**
	 * Generate a cell based on the current set of constraints.
	 *
	 */
  virtual Result generateStructure(
    const StructureDescription &  strDesc,
    const RandomCellDescription & cellDesc) const;

private:

	/**
	/* The maximum number of times to attempt to create a structure before giving up.
	/**/
	const u32               myMaxAttempts;

	/**
	 * The generator used the create the cell for the crystal.
	 */
	const ICellGenerator & myCellGenerator;

	::sstbx::common::Structure * const generateStructure(const ::sstbx::common::AbstractFmidCell * const cell) const;

  bool generateUnitCell(
    const RandomCellDescription & cellDesc,
    ::sstbx::common::Structure &  structure) const;

  StructureGenerationOutcome::Value DefaultCrystalGenerator::generateAtomPositions(
	  StructureDescriptionMap & descriptionMap) const;
};

}}

#endif /* DEFAULT_CRYSTAL_GENERATOR_H */
