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

#include <boost/shared_ptr.hpp>

#include "common/AtomSpeciesId.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx { namespace common {

class AtomSpeciesDatabase
{
public:

	AtomSpeciesDatabase();

	virtual void setAll(
    const AtomSpeciesId::Value id,
		const ::std::string & symbol,
		const ::std::string & name);

  virtual const ::std::string * getName(const AtomSpeciesId::Value id) const;
  virtual void setName(const AtomSpeciesId::Value id, const ::std::string & name);

  virtual const ::std::string * getSymbol(const AtomSpeciesId::Value id) const;
  virtual void setSymbol(const AtomSpeciesId::Value id, const ::std::string & symbol);

  virtual const AtomSpeciesId::Value getIdFromSymbol(const std::string & symbol) const;

	static AtomSpeciesDatabase & inst();

protected:

  typedef ::std::map<AtomSpeciesId::Value, ::std::string> SpeciesString;

  typedef ::std::pair<AtomSpeciesId::Value, ::std::string> SpeciesStringPair;

	SpeciesString	myNames;
	SpeciesString	mySymbols;

private:

  static ::boost::shared_ptr<AtomSpeciesDatabase>  INSTANCE;

};

}}

#endif /* ATOM_SPECIES_DATABASE_H */
