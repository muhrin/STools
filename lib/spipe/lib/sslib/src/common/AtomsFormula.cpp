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

  return true;
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
  return toString() < rhs.toString();
}

AtomsFormula & AtomsFormula::operator +=(const AtomsFormula rhs)
{
  BOOST_FOREACH(Formula::const_reference e, rhs)
  {
    myFormula[e.first] += e.second;
  }
  return *this;
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
  ::std::pair<int, int> fracNum(0, 0);
  int num;
  BOOST_FOREACH(Formula::const_reference e, formula)
  {
    num = numberOf(e.first);
    if(num == 0) // Don't contain any of those, return 0
      return ::std::pair<int, int>(0, 0);

    if(fracNum.first == 0)
    { // Set the fractional number that this formula contains
      fracNum.first = num;
      fracNum.second = e.second;
    }
    else if(fracNum.first != num || fracNum.second != e.second)
      return ::std::pair<int, int>(0, 0);
  }

  return fracNum;
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

::std::ostream & operator <<(::std::ostream & os, const AtomsFormula & formula)
{
  formula.print(os);
  return os;
}

}
}
