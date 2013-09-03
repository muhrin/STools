/*
 * AtomSpeciesDatabase.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef ATOM_SPECIES_DATABASE_H
#define ATOM_SPECIES_DATABASE_H

// INCLUDES /////////////////////////////////////////////
#include <map>
#include <string>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include "spl/common/AtomSpeciesId.h"
#include "spl/OptionalTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace common {

class AtomSpeciesDatabase : ::boost::noncopyable
{
public:
  AtomSpeciesDatabase();

  void
  setAll(AtomSpeciesId::Value id, const ::std::string & name,
      const double radius);

  const ::std::string *
  getName(const AtomSpeciesId::Value id) const;
  void
  setName(const AtomSpeciesId::Value id, const ::std::string & name);

  OptionalDouble
  getRadius(const AtomSpeciesId::Value id) const;
  void
  setRadius(const AtomSpeciesId::Value id, const double radius);

  OptionalDouble
  getSpeciesPairDistance(AtomSpeciesId::Value s1, AtomSpeciesId::Value s2) const;
  void
  setSpeciesPairDistance(const AtomSpeciesId::Value & s1,
      const AtomSpeciesId::Value & s2, const double dist);

protected:
  typedef ::std::map< AtomSpeciesId::Value, ::std::string> SpeciesString;
  typedef ::std::map< AtomSpeciesId::Value, double> SpeciesDouble;
  typedef ::std::map< AtomSpeciesId::Value, SpeciesDouble> SpeciesPairDistances;

  SpeciesString myNames;
  SpeciesDouble myRadii;
  SpeciesPairDistances mySpeciesPairDistances;
};

}
}

#endif /* ATOM_SPECIES_DATABASE_H */
