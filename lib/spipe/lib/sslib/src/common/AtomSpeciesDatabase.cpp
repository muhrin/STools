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

namespace sstbx {
namespace common {


AtomSpeciesDatabase::AtomSpeciesDatabase()
{
  setAll(AtomSpeciesId::H,    "H",  "Hydrogen",   0.25);
  setAll(AtomSpeciesId::HE,   "He", "Helium",     0.28);
  setAll(AtomSpeciesId::LI,   "Li", "Lithium",    1.45);
  setAll(AtomSpeciesId::BE,   "Be", "Beryllium",  1.05);
  setAll(AtomSpeciesId::B,    "B",  "Boron",      0.85);
  setAll(AtomSpeciesId::C,    "C",  "Carbon",     0.7);
  setAll(AtomSpeciesId::N,    "N",  "Nitrogen",   0.65);
  setAll(AtomSpeciesId::O,    "O",  "Oxygen",     0.6);
  setAll(AtomSpeciesId::F,    "F",  "Flourine",   0.5);
  setAll(AtomSpeciesId::NE,   "Ne", "Neon",       0.58);
	setAll(AtomSpeciesId::NA,   "Na", "Sodium",     1.8);
  setAll(AtomSpeciesId::MG,   "Mg", "Magnesium",  1.6);
  setAll(AtomSpeciesId::AL,   "Al", "Aluminium",  1.43);
  setAll(AtomSpeciesId::SI,   "Si", "Silicon",    1.11);
  setAll(AtomSpeciesId::P,    "P",  "Phosphorus", 1.07);
	setAll(AtomSpeciesId::CL,   "Cl", "Chlorine",   1.0);
  setAll(AtomSpeciesId::TI,   "Ti", "Titanium",   1.47);
  setAll(AtomSpeciesId::FE,   "Fe", "Iron",       1.26);
  setAll(AtomSpeciesId::NI,   "Ni", "Nickel",     1.24);
  setAll(AtomSpeciesId::AS,   "As", "Arsenic",    1.19);
  setAll(AtomSpeciesId::SR,   "Sr", "Strontium",  2.15);
  setAll(AtomSpeciesId::IN,   "In", "Indium",     1.67);
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

::boost::optional<double> AtomSpeciesDatabase::getRadius(const AtomSpeciesId::Value id) const
{
  ::boost::optional<double> rad;
	SpeciesDouble::const_iterator it = myRadii.find(id);
	if(it != myRadii.end())
    rad.reset(it->second);
	return rad;
}

void AtomSpeciesDatabase::setRadius(const AtomSpeciesId::Value id, const double radius)
{
  myRadii[id] = radius;
}

void AtomSpeciesDatabase::setAll(
  AtomSpeciesId::Value id,
	const ::std::string & symbol,
	const ::std::string & name,
  const double radius)
{
	setSymbol(id, symbol);
	setName(id, name);
  setRadius(id, radius);
}


}
}
