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

	DefaultCrystalGenerator(const ICellGenerator &	cellGenerator, const bool useExtrudeMethod = false);
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
  /* This option turns on using a method that extrudes any overlapping atoms after an initial
  /* random configuration has been generated.  Otherwise random configuration will be generated
  /* and checked for overlap, if an overlap is detected the configuration is thrown away and
  /* a new one generated.
  /**/
  const bool              myUseExtrudeMethod;

	/**
	 * The generator used the create the cell for the crystal.
	 */
	const ICellGenerator & myCellGenerator;

	::sstbx::common::Structure * const generateStructure(const ::sstbx::common::AbstractFmidCell * const cell) const;

  bool generateUnitCell(
    const RandomCellDescription & cellDesc,
    ::sstbx::common::Structure &  structure) const;

  StructureGenerationOutcome::Value generateAtomPositions(
	  StructureDescriptionMap & descriptionMap) const;
};

}}

#endif /* DEFAULT_CRYSTAL_GENERATOR_H */
