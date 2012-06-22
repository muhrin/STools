/*
 * DefaultCrystalGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef DEFAULTCRYSTALGENERATOR_H
#define DEFAULTCRYSTALGENERATOR_H

// INCLUDES /////////////////////////////////////////////////////
#include "ICrystalStructureGenerator.h"

// FORWARD DECLARES /////////////////////////////////////////////
namespace sstbx
{
	namespace common
	{
		template<typename FloatType>
		class AbstractFmidCell;
		class AtomGroup;
		class Structure;
	}
	namespace build_cell
	{

		class AbstractConstraintDescription;
		class AtomGroupDescription;
		template <typename FloatType>
		class ICellGenerator;
    template <typename FloatType>
    class RandomCellDescription;
		class StructureBuilder;
		class StructureDescription;
	}
}

namespace sstbx { namespace build_cell {

class DefaultCrystalGenerator : public ICrystalStructureGenerator
{
public:

  enum GenerationStatus { SUCCESS, FAILED_MAX_ATTEMPTS, NOT_STARTED };

	DefaultCrystalGenerator(const ICellGenerator<double> &	cellGenerator);

	virtual ~DefaultCrystalGenerator();

	/**
	 * Generate a cell based on the current set of constraints.
	 *
	 */
	virtual ::sstbx::common::Structure * const generateStructure(
    const StructureDescription & strDesc,
    const RandomCellDescription<double> & cellDesc) const;

private:

	/**
	/* The maximum number of times to attempt to create a structure before giving up.
	/**/
	int maxAttempts;

	/**
	 * The generator used the create the cell for the crystal.
	 */
	const ICellGenerator<double> & myCellGenerator;

	::sstbx::common::Structure * const generateStructure(const ::sstbx::common::AbstractFmidCell<double> * const cell) const;


	GenerationStatus generateAtomGroupPositions(
		const StructureBuilder & builder,
		::sstbx::common::AtomGroup & atomGroup)
		const;

	 bool positionGroupsAndAtoms(
		const StructureBuilder & builder,
		::sstbx::common::AtomGroup & atomGroup)
		const;

	bool checkConstraints(
		const StructureBuilder & builder,
		const ::sstbx::common::AtomGroup & atomGroup)
		const;

};

}}

#endif /* DEFAULTCRYSTALGENERATOR_H */
