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

// FORWARD DECLARES ///////////
namespace sstbx {
namespace build_cell {
class AtomConstraintDescription;
}
}

namespace sstbx {
namespace build_cell {

class AtomsDescription
{
public:

	AtomsDescription();
	AtomsDescription(
    const common::AtomSpeciesId::Value species,
    const size_t count = 1);
	virtual ~AtomsDescription() {}

	const common::AtomSpeciesId::Value & getSpecies() const;
	void setSpecies(const common::AtomSpeciesId::Value species);

	size_t getCount() const;
	void setCount(const size_t newCount);

  const OptionalDouble & getRadius() const;
  void setRadius(const OptionalDouble radius);

  const OptionalArmaVec3 & getPosition() const;
  void setPosition(const OptionalArmaVec3 & pos);

private:

  ::sstbx::common::AtomSpeciesId::Value	mySpecies;
  OptionalDouble myRadius;
  OptionalArmaVec3 myPosition;
	size_t myCount;

};


}
}

#endif /* ATOMS_DESCRIPTION_H */
