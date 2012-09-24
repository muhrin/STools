/*
 * Structure.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef STRUCTURE_H
#define STRUCTURE_H

// INCLUDES ///////////////////////////////////////////////
#include "SSLib.h"

#include <memory>

#include <boost/ptr_container/ptr_vector.hpp>

#include <armadillo>

#include "common/Atom.h"
#include "common/AtomSpeciesId.h"
#include "common/DistanceCalculatorDelegator.h"
#include "common/Types.h"
#include "utility/HeterogeneousMap.h"

namespace sstbx {

// FORWARD DECLARATIONS ////////////////////////////////////
namespace utility {
class HeterogeneousMap;
}
namespace common {
class DistanceCalculator;
class UnitCell;

class Structure
{
public:

	Structure(UnitCellPtr cell = UnitCellPtr());

	const std::string & getName() const;
	void setName(const std::string & name);

	// UNIT CELL /////////////////////////////////////////

	UnitCell * getUnitCell();
	const UnitCell * getUnitCell() const;

	/** Set the unit cell to be used by the structure. */
	void setUnitCell(const UnitCellPtr cell);

	// ATOMS ///////////////////////////////////////////////

  size_t getNumAtoms() const;

  Atom & getAtom(const size_t idx);
  const Atom & getAtom(const size_t idx) const;

  Atom & newAtom(const AtomSpeciesId::Value species);
  Atom & newAtom(const Atom & toCopy);
	bool removeAtom(const Atom & atom);
  size_t clearAtoms();

  void getAtomPositions(::arma::mat & posMtx) const;
  void setAtomPositions(const ::arma::mat & posMtx);

  void getAtomSpecies(::std::vector<AtomSpeciesId::Value> & species) const;
  size_t getNumAtomsOfSpecies(const AtomSpeciesId::Value species) const;

  const DistanceCalculator & getDistanceCalculator() const;

  utility::HeterogeneousMap & getProperties();
  const utility::HeterogeneousMap & getProperties() const;

  bool makePrimitive();

  UniquePtr<Structure>::Type getPrimitiveCopy() const;

private:

  typedef ::boost::ptr_vector<Atom> AtomsContainer;

	void atomMoved(const Atom & atom) const;

  inline void unitCellChanged() const
  { myDistanceCalculator.unitCellChanged(); }

	/** The name of this structure, set by calling code */
	std::string		  myName;

	/** The unit cell for this crystal structure. */
	UnitCellPtr     myCell;

  size_t          myNumAtoms;

	/** The atoms contained in this group */
	AtomsContainer  myAtoms;

  utility::HeterogeneousMap  myProperties;

	/**
	/* Flag to indicate whether the structure has changed since
	/* the last time that all atom positions were requested
	/**/
	mutable bool			    myAtomPositionsCurrent;

  mutable ::arma::mat		myAtomPositionsBuffer;

  mutable DistanceCalculatorDelegator  myDistanceCalculator;

  friend class Atom;
  friend class UnitCell;
};

}}

#endif /* STRUCTURE_H */
