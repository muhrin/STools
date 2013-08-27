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

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace common {

class AtomSpeciesDatabase : ::boost::noncopyable
{
public:

  AtomSpeciesDatabase();

  void setAll(
    AtomSpeciesId::Value id,
    const ::std::string & name,
    const double radius
  );

  const ::std::string * getName(const AtomSpeciesId::Value id) const;
  void setName(const AtomSpeciesId::Value id, const ::std::string & name);

  ::boost::optional<double> getRadius(const AtomSpeciesId::Value id) const;
  void setRadius(const AtomSpeciesId::Value id, const double radius);

protected:

  typedef ::std::map<AtomSpeciesId::Value, ::std::string> SpeciesString;
  typedef ::std::map<AtomSpeciesId::Value, double> SpeciesDouble;

  SpeciesString myNames;
  SpeciesDouble myRadii;

};

}}

#endif /* ATOM_SPECIES_DATABASE_H */
