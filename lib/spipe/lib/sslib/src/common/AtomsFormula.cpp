/*
 * AtomsFormula.cpp
 *
 *  Created on: Jul 20, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES ///////////////
#include "common/AtomsFormula.h"

#include <boost/foreach.hpp>

#include "SSLibAssert.h"

namespace sstbx {
namespace common {

AtomsFormula::AtomsFormula(const ::std::string & species)
{
  myFormula[species] = 1;
}

AtomsFormula::AtomsFormula(const ::std::string & species, const int number)
{
  myFormula[species] = number;
}

bool AtomsFormula::fromString(const ::std::string & str)
{
  SSLIB_DIE_NOT_IMPLEMENTED();

  return false;
}

bool AtomsFormula::isEmpty() const
{
  return myFormula.empty();
}

bool AtomsFormula::operator ==(const AtomsFormula & rhs) const
{
  BOOST_FOREACH(Formula::const_reference e, myFormula)
  {
    if(rhs.numberOf(e.first) != e.second)
      return false;
  }
  return true;
}

AtomsFormula & AtomsFormula::operator +=(const AtomsFormula rhs)
{
  BOOST_FOREACH(Formula::const_reference e, rhs)
  {
    myFormula[e.first] += e.second;
  }
  return *this;
}

AtomsFormula::const_iterator AtomsFormula::begin() const
{
  return myFormula.begin();
}

AtomsFormula::const_iterator AtomsFormula::end() const
{
  return myFormula.end();
}

bool AtomsFormula::contains(const ::std::string & species) const
{
  return myFormula.find(species) != myFormula.end();
}

int AtomsFormula::numberOf(const std::string & species) const
{
  const Formula::const_iterator it = myFormula.find(species);
  if(it == myFormula.end())
    return 0;

  return it->second;
}

void AtomsFormula::print(::std::ostream & os) const
{
  BOOST_FOREACH(Formula::const_reference e, myFormula)
  {
    os << e.first;
    if(e.second != 1)
      os << e.second;
  }
}

::std::ostream & operator <<(::std::ostream & os, const AtomsFormula & formula)
{
  formula.print(os);
  return os;
}

}
}
