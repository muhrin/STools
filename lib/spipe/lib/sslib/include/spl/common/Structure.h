/*
 * Structure.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef STRUCTURE_H
#define STRUCTURE_H

// INCLUDES ///////////////////////////////////////////////
#include "spl/SSLib.h"

#include <map>
#include <memory>
#include <ostream>

#include <boost/lexical_cast.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <armadillo>

#include "spl/common/Atom.h"
#include "spl/common/AtomSpeciesId.h"
#include "spl/common/DistanceCalculatorDelegator.h"
#include "spl/common/StructureProperties.h"
#include "spl/common/Types.h"
#include "spl/common/UnitCell.h"
#include "spl/utility/HeterogeneousMap.h"
#include "spl/utility/NamedProperty.h"

std::ostream &
operator<<(std::ostream & os, const spl::common::Structure & p);

std::ostream &
operator<<(std::ostream & os, const spl::common::Structure & p);

namespace spl {
// FORWARD DECLARATIONS ////////////////////////////////////
namespace common {
class AtomsFormula;
class DistanceCalculator;
class UnitCell;

class Structure : public UnitCell::UnitCellListener
{
public:

  typedef utility::NamedProperty< utility::HeterogeneousMap> VisibleProperty;

  explicit
  Structure(UnitCellPtr cell = UnitCellPtr());
  Structure(const Structure & toCopy);
  Structure &
  operator =(const Structure & rhs);
  StructurePtr
  clone() const;

  void
  updateWith(const Structure & structure);

  const std::string &
  getName() const;
  void
  setName(const std::string & name);

  // UNIT CELL /////////////////////////////////////////

  UnitCell *
  getUnitCell();
  const UnitCell *
  getUnitCell() const;

  /** Set the unit cell to be used by the structure. */
  void
  setUnitCell(UnitCellPtr cell);

  // ATOMS ///////////////////////////////////////////////

  size_t
  getNumAtoms() const;

  Atom &
  getAtom(const size_t idx);
  const Atom &
  getAtom(const size_t idx) const;

  Atom &
  newAtom(const AtomSpeciesId::Value species);
  Atom &
  newAtom(const Atom & toCopy);
  bool
  removeAtom(const Atom & atom);
  size_t
  clearAtoms();

  void
  getAtomPositions(::arma::mat & posMtx) const;
  void
  getAtomPositions(::arma::subview< double> & posMtx) const;
  void
  setAtomPositions(const ::arma::mat & posMtx);

  void
  getAtomSpecies(::std::vector< AtomSpeciesId::Value> & species) const;
  size_t
  getNumAtomsOfSpecies(const AtomSpeciesId::Value species) const;
  AtomsFormula
  getComposition() const;

  const DistanceCalculator &
  getDistanceCalculator() const;

  template< typename T>
    T *
    getProperty(const utility::Key< T> & key);

  template< typename T>
    const T *
    getProperty(const utility::Key< T> & key) const;

  template< typename T>
    void
    setProperty(utility::Key< T> & key, const T & value);

  template< typename T>
    void
    setPropertyFromString(utility::Key< T> & key, const ::std::string & value);

  template< typename T>
    bool
    eraseProperty(utility::Key< T> & key);

  ::boost::optional< ::std::string>
  getVisibleProperty(const VisibleProperty & property) const;
  void
  setVisibleProperty(VisibleProperty & property, const ::std::string & value);

  bool
  makePrimitive();

  UniquePtr< Structure>::Type
  getPrimitiveCopy() const;

  void
  scale(const double scaleFactor);

  void
  print(::std::ostream & os) const;

  virtual void
  onUnitCellChanged(UnitCell & unitCell);
  virtual void
  onUnitCellVolumeChanged(UnitCell & unitCell, const double oldVol,
      const double newVol);

private:

  typedef ::boost::ptr_vector< Atom> AtomsContainer;

  void
  atomMoved(const Atom & atom) const;

  void
  updatePosBuffer() const;

  /** The name of this structure, set by calling code */
  std::string myName;

  /** The unit cell for this crystal structure. */
  UnitCellPtr myCell;

  /** The atoms contained in this group */
  AtomsContainer myAtoms;

  utility::HeterogeneousMap myTypedProperties;

  /**
   /* Flag to indicate whether the structure has changed since
   /* the last time that all atom positions were requested
   /**/
  mutable bool myAtomPositionsCurrent;

  mutable ::arma::mat myAtomPositionsBuffer;

  mutable DistanceCalculatorDelegator myDistanceCalculator;

  friend class Atom;
};

template< typename T>
  T *
  Structure::getProperty(const utility::Key< T> & key)
  {
    return myTypedProperties.find(key);
  }

template< typename T>
  const T *
  Structure::getProperty(const utility::Key< T> & key) const
  {
    return myTypedProperties.find(key);
  }

template< typename T>
  void
  Structure::setProperty(utility::Key< T> & key, const T & value)
  {
    myTypedProperties[key] = value;
  }

template< typename T>
  void
  Structure::setPropertyFromString(utility::Key< T> & key,
      const ::std::string & value)
  {
    setProperty(key, ::boost::lexical_cast< T>(value));
  }

template< typename T>
  bool
  Structure::eraseProperty(utility::Key< T> & key)
  {
    return myTypedProperties.erase(key) != 0;
  }

}
}

#endif /* STRUCTURE_H */
