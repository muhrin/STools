/*
 * SpeciesPair.h
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */


// INCLUDES /////////////////
#include "spl/build_cell/SpeciesPair.h"


namespace spl {
namespace build_cell {

SpeciesPair::SpeciesPair(const ::std::string & species1, const ::std::string & species2)
{
  if(species1 < species2)
  {
    myPair.first = species1;
    myPair.second = species2;
  }
  else
  {
    myPair.first = species2;
    myPair.second = species1;
  }
}

SpeciesPair::SpeciesPair(const SpeciesPair & toCopy)
{
  *this = toCopy;
}

const ::std::string & SpeciesPair::first() const
{
  return myPair.first;
}

const ::std::string & SpeciesPair::second() const
{
  return myPair.second;
}

SpeciesPair & SpeciesPair::operator =(const SpeciesPair & rhs)
{
  myPair = rhs.myPair;
  return *this;
}

bool SpeciesPair::operator ==(const SpeciesPair & rhs) const
{
  return myPair == rhs.myPair;
}

bool SpeciesPair::operator <(const SpeciesPair & rhs) const
{
  return toString() < rhs.toString();
}

::std::string SpeciesPair::toString() const
{
  return myPair.first + myPair.second;
}

}
}
