/*
 * Structure.cpp
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES /////////////////////////////////////
#include "common/Structure.h"

#include <vector>

#include "SSLibAssert.h"
#include "common/Atom.h"
#include "common/Types.h"
#include "common/UnitCell.h"

#ifdef _MSC_VER
// Disable warning about passing this pointer to DistanceCalculator in initialisation list
#pragma warning( disable : 4355 )
#endif

namespace sstbx {
namespace common {

Structure::Structure(UnitCellPtr cell):
myCell(cell),
myAtomPositionsCurrent(false),
myNumAtoms(0),
myDistanceCalculator(*this)
{}

const std::string & Structure::getName() const
{
	return myName;
}

void Structure::setName(const std::string & name)
{
	myName = name;
}

UnitCell * Structure::getUnitCell()
{
	return myCell.get();
}

const UnitCell * Structure::getUnitCell() const
{
	return myCell.get();
}

void Structure::setUnitCell(const UnitCellPtr cell)
{
  if(myCell)
    myCell->setStructure(NULL);

	myCell = cell;
  myCell->setStructure(this);
  myDistanceCalculator.unitCellChanged();
}

size_t Structure::getNumAtoms() const
{
  return myAtoms.size();
}

Atom & Structure::getAtom(const size_t idx)
{
  SSE_ASSERT(idx < getNumAtoms());

  return myAtoms[idx];
}

const Atom & Structure::getAtom(const size_t idx) const
{
  SSE_ASSERT(idx < getNumAtoms());

  return myAtoms[idx];
}

Atom & Structure::newAtom(const AtomSpeciesId::Value species)
{
  Atom * const atom = new Atom(species, *this, myNumAtoms++);
  myAtoms.push_back(atom);
  return *atom;
}

Atom & Structure::newAtom(const Atom & toCopy)
{
  Atom * const atom = new Atom(toCopy, *this, ++myNumAtoms);
  myAtoms.push_back(atom);
  return *atom;
}

bool Structure::removeAtom(const Atom & atom)
{
  if(&atom.getStructure() != this)
    return false;

  const size_t index = atom.getIndex();

  myAtoms.erase(myAtoms.begin() + index);
  --myNumAtoms;

  for(size_t i = index; i < myNumAtoms; ++i)
  {
    myAtoms[i].setIndex(i);
  }

  return true;
}

void Structure::getAtomPositions(::arma::mat & posMtx) const
{
	// Do we need to update the buffer?
	if(!myAtomPositionsCurrent)
	{
		myAtomPositionsBuffer.reset();
    myAtomPositionsBuffer.set_size(3, getNumAtoms());
		for(size_t i = 0; i < getNumAtoms(); ++i)
    {
      myAtomPositionsBuffer.col(i) = myAtoms[i].getPosition();
    }
		myAtomPositionsCurrent = true;
	}

	posMtx = myAtomPositionsBuffer;
}

void Structure::setAtomPositions(const ::arma::mat & posMtx)
{
  const size_t numAtoms = getNumAtoms();
  SSLIB_ASSERT(posMtx.n_rows == 3 && posMtx.n_cols == numAtoms);

  for(size_t i = 0; i < numAtoms; ++i)
  {
    myAtoms[i].setPosition(posMtx.col(i));
  }

	// Save the new positions in the buffer
	myAtomPositionsBuffer	= posMtx;
	myAtomPositionsCurrent	= true;
}

void Structure::getAtomSpecies(::std::vector<AtomSpeciesId::Value> & species) const
{
  const size_t numAtoms = getNumAtoms();
  species.resize(numAtoms);

  for(size_t i = 0; i < numAtoms; ++i)
  {
    species[i] = myAtoms[i].getSpecies();
  }
}

size_t Structure::getNumAtomsOfSpecies(const AtomSpeciesId::Value species) const
{
  const size_t numAtoms = getNumAtoms();
  size_t num = 0;
  for(size_t i = 0; i < numAtoms; ++i)
  {
    if(myAtoms[i].getSpecies() == species)
      ++num;
  }
  return num;
}

const DistanceCalculator & Structure::getDistanceCalculator() const
{
  return myDistanceCalculator;
}

::sstbx::utility::HeterogeneousMap & Structure::getProperties()
{
  return myProperties;
}

const ::sstbx::utility::HeterogeneousMap & Structure::getProperties() const
{
  return myProperties;
}

void Structure::atomMoved(const Atom & atom) const
{
  // Atom has moved so the buffer is not longer current
  myAtomPositionsCurrent = false;
}

}
}
