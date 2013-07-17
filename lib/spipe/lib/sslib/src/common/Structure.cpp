/*
 * Structure.cpp
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES /////////////////////////////////////
#include "common/Structure.h"

#include <vector>

#include <boost/foreach.hpp>

extern "C"
{
#  include <spglib/spglib.h>
}

#include "SSLibAssert.h"
#include "common/Atom.h"
#include "common/Types.h"
#include "common/UnitCell.h"
#include "utility/IndexingEnums.h"

#ifdef _MSC_VER
// Disable warning about passing this pointer to DistanceCalculator in initialisation list
#  pragma warning( push )
#  pragma warning( disable : 4355 )
#endif

namespace sstbx {
namespace common {

class MatchSpecies : public std::unary_function<const Atom &, bool>
{
public:
  MatchSpecies(const AtomSpeciesId::Value toMatch):
  mySpecies(toMatch) {}

  bool operator() (const Atom & atom) {return atom.getSpecies() == mySpecies;}

private:
  const AtomSpeciesId::Value mySpecies;
};

Structure::Structure(UnitCellPtr cell):
myAtomPositionsCurrent(false),
myNumAtoms(0),
myDistanceCalculator(*this)
{
  setUnitCell(cell);
}

Structure::Structure(const Structure & toCopy):
myDistanceCalculator(*this)
{
  // Use the equals operator so we don't duplicate code
  *this = toCopy;
}

Structure & Structure::operator =(const Structure & rhs)
{
  myName = rhs.myName;

  // Copy over the unit cell (if exists)
  if(rhs.myCell.get())
    setUnitCell(rhs.myCell->clone());

  // Copy over properties
  myTypedProperties = rhs.myTypedProperties;

  // Copy over the atoms
  clearAtoms();
  BOOST_FOREACH(const Atom & atom, rhs.myAtoms)
  {
    newAtom(atom);
  }

  return *this;
}

StructurePtr Structure::clone() const
{
  return StructurePtr(new Structure(*this));
}

void Structure::updateWith(const Structure & structure)
{
  // Update the unit cell
  if(structure.getUnitCell())
    setUnitCell(structure.getUnitCell()->clone());
  else
    setUnitCell(UnitCellPtr());

  // Update the atoms
  clearAtoms();
  BOOST_FOREACH(const Atom & atom, structure.myAtoms)
  {
    newAtom(atom);
  }

  // Update the properties
  myTypedProperties.insert(structure.myTypedProperties, true);
}

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

void Structure::setUnitCell(UnitCellPtr cell)
{
  if(myCell.get() == cell.get())
    return;

  if(myCell.get())
    myCell->removeListener(*this);

  myCell = cell;
  myCell->addListener(*this);
  myDistanceCalculator.unitCellChanged();
}

size_t Structure::getNumAtoms() const
{
  return myAtoms.size();
}

Atom & Structure::getAtom(const size_t idx)
{
  SSLIB_ASSERT(idx < getNumAtoms());

  return myAtoms[idx];
}

const Atom & Structure::getAtom(const size_t idx) const
{
  SSLIB_ASSERT(idx < getNumAtoms());

  return myAtoms[idx];
}

Atom & Structure::newAtom(const AtomSpeciesId::Value species)
{
  myAtomPositionsCurrent = false;
  Atom * const atom = new Atom(species, *this, myNumAtoms++);
  myAtoms.push_back(atom);
  return *atom;
}

Atom & Structure::newAtom(const Atom & toCopy)
{
  myAtomPositionsCurrent = false;
  return *myAtoms.insert(myAtoms.end(), new Atom(toCopy, *this, ++myNumAtoms));
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

  myAtomPositionsCurrent = false;
  return true;
}

size_t Structure::clearAtoms()
{
  const size_t previousNumAtoms = myNumAtoms;

  myAtoms.clear();

  myNumAtoms = 0;
  myAtomPositionsCurrent = false;
  return previousNumAtoms;
}

void Structure::getAtomPositions(::arma::mat & posMtx) const
{
	// Do we need to update the buffer?
	if(!myAtomPositionsCurrent)
    updatePosBuffer();

	posMtx = myAtomPositionsBuffer;
}

void Structure::getAtomPositions(::arma::subview<double> & posMtx) const
{
	// Do we need to update the buffer?
	if(!myAtomPositionsCurrent)
    updatePosBuffer();

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
  return ::std::count_if(myAtoms.begin(), myAtoms.end(), MatchSpecies(species));
}

Structure::Composition Structure::getComposition() const
{
  Composition comp;

  Composition::iterator it;
  BOOST_FOREACH(const Atom & atom, myAtoms)
  {
    it = comp.find(atom.getSpecies());
    if(it == comp.end())
      comp[atom.getSpecies()] = 1;
    else
      ++it->second;
  }
  return comp;
}

const DistanceCalculator & Structure::getDistanceCalculator() const
{
  return myDistanceCalculator;
}

::boost::optional< ::std::string>
Structure::getVisibleProperty(const VisibleProperty & property) const
{
  return property.getValue(myTypedProperties);
}

void Structure::setVisibleProperty(VisibleProperty & property, const ::std::string & value)
{
  property.setValue(myTypedProperties, value);
}

bool Structure::makePrimitive()
{
  if(myNumAtoms > 0 && myCell.get())
  {
    double lattice[3][3];
    const::arma::mat33 & orthoMtx = myCell->getOrthoMtx();
    for(size_t i = 0; i < 3; ++i)
    {
      for(size_t j = 0; j < 3; ++j)
      {
        // Row-major = column-major
        lattice[i][j] = orthoMtx(i, j);
      }
    }

    double (*positions)[3] = new double[myNumAtoms][3];
    ::arma::mat posMtx;
    getAtomPositions(posMtx);
    myCell->cartsToFracInplace(posMtx);
    for(size_t i = 0; i < myNumAtoms; ++i)
    {
      for(size_t j = 0; j < 3; ++j)
      {
        // Row-major = column-major
        positions[i][j] = posMtx(j, i);
      }
    }

    ::std::vector<AtomSpeciesId::Value> speciesVec;
    ::std::vector<AtomSpeciesId::Value> speciesIdxVec;
    getAtomSpecies(speciesVec);
    ::std::map<common::AtomSpeciesId::Value, int> speciesIndices;
    int idx = 0;
    BOOST_FOREACH(const common::AtomSpeciesId::Value & speciesId, speciesVec)
    {
      if(speciesIndices.insert(::std::make_pair(speciesId, idx)).second == true)
      {
        speciesIdxVec.push_back(speciesId);
        ++idx;
      }
    }

    ::boost::scoped_array<int> species(new int[speciesVec.size()]);
    for(size_t i = 0; i < speciesVec.size(); ++i)
    {
      species[i] = speciesIndices[speciesVec[i]];
    }

    // Try to find the primitive unit cell
    const size_t newNumAtoms = (size_t)spg_find_primitive(lattice, positions, species.get(), myNumAtoms, 0.05);

    if(newNumAtoms != 0 && newNumAtoms < myNumAtoms)
    {
      // First deal with lattice
      ::arma::mat33 newLattice;
      for(size_t i = 0; i < 3; ++i)
      {
        for(size_t j = 0; j < 3; ++j)
        {
          newLattice(i, j) = lattice[i][j];
        }
      }

      myCell->setOrthoMtx(newLattice);

      // Now deal with atoms
      clearAtoms();

      Atom * atom;
      ::arma::vec3 pos;

      for(size_t i = 0; i < newNumAtoms; ++i)
      {
        atom = &newAtom(speciesIdxVec[species[i]]);
        pos << positions[i][0] << ::arma::endr
          << positions[i][1] << ::arma::endr
          << positions[i][2] << ::arma::endr;
        atom->setPosition(myCell->fracWrapToCartInplace(pos));
      }
      delete [] positions;
      return true;
    }
  }

  return false;
}

UniquePtr<Structure>::Type Structure::getPrimitiveCopy() const
{
  UniquePtr<Structure>::Type structure(new Structure(*this));
  structure->makePrimitive();
  return structure;
}

void Structure::scale(const double scaleFactor)
{
  UnitCell * const unitCell = getUnitCell();

  if(unitCell)
  {
    const double volume = unitCell->getVolume();
    ::arma::mat atomPositions;
    getAtomPositions(atomPositions);
    unitCell->cartsToFracInplace(atomPositions);                    // Generate fractional positions
    unitCell->setVolume(volume * scaleFactor);                      // Scale the unit cell
    setAtomPositions(unitCell->fracsToCartInplace(atomPositions));  // Use the scaled cell to convert back to cart
  }
  else
  {
    // TODO: Implement cluster scaling
  }
}

void Structure::print(::std::ostream & os) const
{
  using namespace utility::cell_params_enum;

  os << "Structure";
  if(!myName.empty())
    os << " " << myName;
  os << ":" << std::endl;

  const UnitCell * const unitCell = getUnitCell();
  if(unitCell)
  {
    const double (&params)[6] = unitCell->getLatticeParams();
    os << "Unit cell: " << params[A] << " " << params[B] << " " << params[C]
      << params[ALPHA] << " " << params[BETA] << " " << params[GAMMA] << std::endl;
  }

  ::arma::vec3 pos;
  os << "Atoms" << std::endl;
  for(size_t i = 0; i < getNumAtoms(); ++i)
  {
    const Atom & atom = getAtom(i);
    // Species
    //os << atom.getSpecies().toString();
    // Positions
    pos = atom.getPosition();
    for(size_t i = 0; i < 3; ++i)
      os << " " << pos(i);
    os << std::endl;
  }
}

void Structure::onUnitCellChanged(UnitCell & unitCell)
{
  myDistanceCalculator.unitCellChanged();
}

void Structure::onUnitCellVolumeChanged(UnitCell & unitCell, const double oldVol,
    const double newVol)
{
  myDistanceCalculator.unitCellChanged();
}

void Structure::atomMoved(const Atom & atom) const
{
  // Atom has moved so the buffer is not longer current
  myAtomPositionsCurrent = false;
}

void Structure::updatePosBuffer() const
{
	myAtomPositionsBuffer.reset();
  myAtomPositionsBuffer.set_size(3, getNumAtoms());
	for(size_t i = 0; i < getNumAtoms(); ++i)
  {
    myAtomPositionsBuffer.col(i) = myAtoms[i].getPosition();
  }
	myAtomPositionsCurrent = true;
}

} // namespace common
} // namespace sstbx

// Global namespace
std::ostream & operator<<(
  std::ostream & os,
  const sstbx::common::Structure & structure)
{
  structure.print(os);
  return os;
}
