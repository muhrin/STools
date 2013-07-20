/*
 * AtomsFormula.h
 *
 *  Created on: Jul 20, 2013
 *      Author: Martin Uhrin
 */

#ifndef ATOM_FORMULA_H
#define ATOM_FORMULA_H

// INCLUDES ///////////////////////////////////
#include "SSLib.h"

#include <map>
#include <ostream>
#include <string>

namespace sstbx {
namespace common {

// FORWARD DECLARES ///////////////////////////

class AtomsFormula
{
  typedef ::std::map< ::std::string, int> Formula;
  typedef Formula::value_type Entry;
public:
  typedef Formula::value_type value_type;
  typedef Formula::const_reference const_reference;
  typedef Formula::const_iterator const_iterator;
  typedef const_iterator iterator; // Only const iteration allowed

  AtomsFormula() {}
  AtomsFormula(const ::std::string & species);
  AtomsFormula(const ::std::string & species, const int number);

  bool fromString(const ::std::string & str);

  bool isEmpty() const;

  bool operator ==(const AtomsFormula & rhs) const;

  AtomsFormula & operator +=(const AtomsFormula rhs);

  const_iterator begin() const;
  const_iterator end() const;

  bool contains(const ::std::string & species) const;
  int numberOf(const std::string & species) const;

  void clear();

  void print(::std::ostream & os) const;

private:

  Formula myFormula;
};

::std::ostream & operator <<(::std::ostream & os, const AtomsFormula & formula);

}
}

#endif /* ATOM_FORMULA_H */
