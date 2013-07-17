/*
 * AtomsDescription.h
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#ifndef ATOMS_DESCRIPTION_H
#define ATOMS_DESCRIPTION_H

// INCLUDES ///////////////////

#include <string>

#include <boost/ptr_container/ptr_map.hpp>

#include "OptionalTypes.h"
#include "common/AtomSpeciesId.h"
#include "utility/Range.h"

// FORWARD DECLARES ///////////
namespace sstbx
{
namespace build_cell
{
class AtomConstraintDescription;
}
}

namespace sstbx
{
namespace build_cell
{

class AtomsDescription
{
public:
  typedef utility::Range< int> CountRange;

  AtomsDescription();
  AtomsDescription(const common::AtomSpeciesId::Value species,
      const size_t count = 1);
  AtomsDescription(const common::AtomSpeciesId::Value species,
      const CountRange count);
  virtual
  ~AtomsDescription()
  {
  }

  const common::AtomSpeciesId::Value &
  getSpecies() const;
  void
  setSpecies(const common::AtomSpeciesId::Value species);

  CountRange
  getCount() const;
  void
  setCount(const int count);
  void
  setCount(const CountRange count);

  const OptionalDouble &
  getRadius() const;
  void
  setRadius(const OptionalDouble radius);

  const OptionalArmaVec3 &
  getPosition() const;
  void
  setPosition(const OptionalArmaVec3 & pos);

private:
  common::AtomSpeciesId::Value mySpecies;
  OptionalDouble myRadius;
  OptionalArmaVec3 myPosition;
  CountRange myCount;
};

}
}

#endif /* ATOMS_DESCRIPTION_H */
