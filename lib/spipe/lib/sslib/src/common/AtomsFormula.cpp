/*
 * AtomsFormula.cpp
 *
 *  Created on: Jul 20, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES ///////////////
#include "common/AtomsFormula.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include "SSLibAssert.h"
#include "math/NumberAlgorithms.h"

#include <iostream> // TODO: TEMP, DELETE

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
  static const boost::regex FORM_EXPRESSION("([[:upper:]][[:lower:]]*)([[:digit:]]*)");

  if(str.empty())
    return false;

  ::std::string::const_iterator start = str.begin();
  const ::std::string::const_iterator end = str.end();
  boost::match_results< ::std::string::const_iterator> match;
  boost::match_flag_type flags = boost::match_default;
  while(::boost::regex_search(start, end, match, FORM_EXPRESSION, flags))
  {
    // Get the species
    Entry entry(::std::string(match[1].first, match[1].second), 1);

    // Get the number
    ::std::string numStr(match[2].first, match[2].second);
    if(!numStr.empty())
    {
      try
      {
        entry.second = ::boost::lexical_cast<int>(numStr);
      }
      catch(const ::boost::bad_lexical_cast & /*e*/)
      {
        return false;
      }
    }
    myFormula.insert(entry);

    // update search position:
    start = match[0].second;
    // update flags:
    flags |= boost::match_prev_avail;
    flags |= boost::match_not_bob;
  }

  return !isEmpty();
}

unsigned int AtomsFormula::reduce()
{
  // Divide the counts by the greatest common divisor
  ::std::vector<unsigned int> counts;
  BOOST_FOREACH(Formula::const_reference e, myFormula)
    counts.push_back(e.second);
  const unsigned int gcd = math::greatestCommonDivisor(counts);
  BOOST_FOREACH(Formula::reference e, myFormula)
    e.second /= static_cast<int>(gcd);
  return gcd;
}

bool AtomsFormula::isEmpty() const
{
  return myFormula.empty();
}

int AtomsFormula::numSpecies() const
{
  return myFormula.size();
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

bool AtomsFormula::operator <(const AtomsFormula & rhs) const
{
  for(Formula::const_iterator it = myFormula.begin(), end = myFormula.end(),
      rhsIt = rhs.myFormula.begin(), rhsEnd = rhs.myFormula.end();
      it != end && rhsIt != rhsEnd; ++it, ++rhsIt)
  {
    if(it->first < rhsIt->first)
      return true;
    else if(it->first > rhsIt->first)
      return false;
    else if(it->second < rhsIt->second)
      return true;
    else if(it->second > rhsIt->second)
      return false;
  }
  // So far they have been the same, maybe they are different lengths?
  return myFormula.size() < rhs.myFormula.size();
}

AtomsFormula & AtomsFormula::operator +=(const AtomsFormula rhs)
{
  BOOST_FOREACH(Formula::const_reference e, rhs)
  {
    myFormula[e.first] += e.second;
  }
  return *this;
}

bool AtomsFormula::remove(const AtomsFormula & toRemove)
{
  return remove(toRemove, 1);
}

bool AtomsFormula::remove(const AtomsFormula & toRemove, const int numToRemove)
{
  // Find out how many of those to remove we have
  const ::std::pair<int, int> num = numberOf(toRemove);

  // Check we have that formula to remove and that we don't have a fractional number of them
  if(num.first == 0 || num.second != 1 || numToRemove > num.first)
    return false;

  Formula::iterator it;
  BOOST_FOREACH(Formula::const_reference e, toRemove)
  {
    it = myFormula.find(e.first);
    it->second -= numToRemove * e.second;
    if(it->second == 0)
      myFormula.erase(it);
  }
  return true;
}

int & AtomsFormula::operator [](const ::std::string & species)
{
  return myFormula[species];
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

bool AtomsFormula::contains(const AtomsFormula & formula) const
{
  return numberOf(formula).first != 0;
}

int AtomsFormula::numberOf(const std::string & species) const
{
  const Formula::const_iterator it = myFormula.find(species);
  if(it == myFormula.end())
    return 0;

  return it->second;
}

::std::pair<int, int> AtomsFormula::numberOf(const AtomsFormula & formula) const
{
  static const ::std::pair<int, int> NOT_FOUND(0, 0);

  ::std::pair<int, int> fracNum(0, 0);
  ::std::pair<int, int> currentFrac;
  BOOST_FOREACH(Formula::const_reference e, formula)
  {
    currentFrac.first = numberOf(e.first);
    currentFrac.second = e.second;
    if(currentFrac.first == 0) // Don't contain any of those, return 0
      return NOT_FOUND;

    simplify(currentFrac);
    if(fracNum.first == 0)
    { // Set the fractional number that this formula contains
      fracNum = currentFrac;
    }
    else if(fracNum != currentFrac)
      return NOT_FOUND;
  }

  return fracNum;
}

int AtomsFormula::numMultiples(const AtomsFormula & formula) const
{
  if(numSpecies() != formula.numSpecies())
    return -1;

  const ::std::pair<int, int> num = numberOf(formula);
  if(num.first == 0 || num.second != 1)
    return -1;

  return num.first;
}

::std::string AtomsFormula::toString() const
{
  ::std::stringstream ss;
  BOOST_FOREACH(Formula::const_reference e, myFormula)
  {
    ss << e.first;
    if(e.second != 1)
      ss << e.second;
  }
  return ss.str();
}

void AtomsFormula::print(::std::ostream & os) const
{
  os << toString();
}

bool AtomsFormula::simplify(::std::pair<int, int> & fraction) const
{
  if(fraction.first == 0 || fraction.second == 0)
    return false;

  const int gcd = math::greatestCommonDivisor(fraction.first, fraction.second);
  if(gcd == 1)
    return false;

  fraction.first = fraction.first / gcd;
  fraction.second = fraction.second / gcd;
  return true;
}

::std::ostream & operator <<(::std::ostream & os, const AtomsFormula & formula)
{
  formula.print(os);
  return os;
}

}
}
