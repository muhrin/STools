/*
 * AtomSpciesDatabase.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/AtomSpeciesDatabase.h"

#include "common/AtomSpeciesId.h"

// NAMESPACES ////////////////////////////////

namespace sstbx { namespace common {

// Declare the singleton instance
::boost::shared_ptr<AtomSpeciesDatabase> AtomSpeciesDatabase::INSTANCE;

AtomSpeciesDatabase::AtomSpeciesDatabase()
{
  setAll(AtomSpeciesId::H, "H", "Hydrogen");
	setAll(AtomSpeciesId::NA, "Na", "Sodium");
	setAll(AtomSpeciesId::CL, "Cl", "Chlorine");
}

const ::std::string * AtomSpeciesDatabase::getName(const AtomSpeciesId::Value id) const
{
	SpeciesString::const_iterator it = myNames.find(id);
	if(it == myNames.end())
		return NULL;
	return &it->second;
}

void AtomSpeciesDatabase::setName(const AtomSpeciesId::Value id, const ::std::string & name)
{
	myNames[id] = name;
}

const ::std::string * AtomSpeciesDatabase::getSymbol(const AtomSpeciesId::Value id) const
{
	SpeciesString::const_iterator it = mySymbols.find(id);
	if(it == mySymbols.end())
		return NULL;
	return &it->second;
}

void AtomSpeciesDatabase::setSymbol(const AtomSpeciesId::Value id, const ::std::string & symbol)
{
	mySymbols[id] = symbol;
}

const AtomSpeciesId::Value AtomSpeciesDatabase::getIdFromSymbol(const std::string & symbol) const
{
  AtomSpeciesId::Value id = AtomSpeciesId::DUMMY;
  for(SpeciesString::const_iterator it = mySymbols.begin(), end = mySymbols.end();
    it != end; ++it)
  {
    if(it->second == symbol)
    {
      id = it->first;
      break;
    }
  }
  return id;
}

void AtomSpeciesDatabase::setAll(
  AtomSpeciesId::Value id,
	const ::std::string & symbol,
	const ::std::string & name)
{
	setSymbol(id, symbol);
	setName(id, name);
}

AtomSpeciesDatabase & AtomSpeciesDatabase::inst()
{
	if(!INSTANCE.get())
	{
		INSTANCE.reset(new AtomSpeciesDatabase());
	}
	return *INSTANCE;
}

}}
