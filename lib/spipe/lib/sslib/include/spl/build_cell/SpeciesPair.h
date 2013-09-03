/*
 * SpeciesPair.h
 *
 *  Created on: Aug 2, 2013
 *      Author: Martin Uhrin
 */

#ifndef SPECIES_PAIR_H
#define SPECIES_PAIR_H

// INCLUDES /////////////////////////////////
#include "spl/SSLib.h"

#include <string>
#include <utility>

// FORWARD DECLARES //////////////////////////

namespace spl {
namespace build_cell {

class SpeciesPair
{
public:
  SpeciesPair(const ::std::string & species1, const ::std::string & species2);
  SpeciesPair(const SpeciesPair & toCopy);

  const ::std::string & first() const;
  const ::std::string & second() const;

  SpeciesPair & operator =(const SpeciesPair & rhs);

  bool operator ==(const SpeciesPair & rhs) const;
  bool operator <(const SpeciesPair & rhs) const;

private:
  ::std::string toString() const;

  ::std::pair< ::std::string, ::std::string> myPair;
};

}
}

#endif /* SPECIES_PAIR_H */
